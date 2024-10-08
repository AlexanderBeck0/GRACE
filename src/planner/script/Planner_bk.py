#!/usr/bin/env python2
import numpy
import numpy as np
from numpy.linalg import inv, norm, eig, det
from scipy.interpolate import interp1d
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from darknet_ros_msgs.msg import Object2D, Object2DArray, RobotCommand
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import TransformException, ConnectivityException, \
    ExtrapolationException
from cv_bridge import CvBridge
import cv2
import tf
from scipy.integrate import dblquad
from scipy.stats import multivariate_normal as mvn
from scipy.stats import norm as svn
from scipy.stats import dirichlet
from skimage.draw import line
from scipy import io
from scipy.special import softmax
import math
from time import time
from threading import Thread, Lock
from datetime import datetime
import actionlib
from ipa_building_msgs.msg import MapSegmentationAction, MapSegmentationGoal
from darknet_ros_msgs.srv import *


def bresenham_supercover_line(pt1, pt2):
    r"""Line drawing algo based
    on http://eugen.dedu.free.fr/projects/bresenham/
    """

    ystep, xstep = 1, 1

    x, y = pt1
    dx, dy = pt2 - pt1

    if dy < 0:
        ystep *= -1
        dy *= -1

    if dx < 0:
        xstep *= -1
        dx *= -1

    line_pts = [[x, y]]

    ddx, ddy = 2 * dx, 2 * dy
    if ddx > ddy:
        errorprev = dx
        error = dx
        for _ in range(int(dx)):
            x += xstep
            error += ddy

            if error > ddx:
                y += ystep
                error -= ddx
                if error + errorprev < ddx:
                    line_pts.append([x, y - ystep])
                elif error + errorprev > ddx:
                    line_pts.append([x - xstep, y])
                else:
                    line_pts.append([x - xstep, y])
                    line_pts.append([x, y - ystep])

            line_pts.append([x, y])

            errorprev = error
    else:
        errorprev = dx
        error = dx
        for _ in range(int(dy)):
            y += ystep
            error += ddx

            if error > ddy:
                x += xstep
                error -= ddy
                if error + errorprev < ddy:
                    line_pts.append([x - xstep, y])
                elif error + errorprev > ddy:
                    line_pts.append([x, y - ystep])
                else:
                    line_pts.append([x - xstep, y])
                    line_pts.append([x, y - ystep])

            line_pts.append([x, y])

            errorprev = error

    return line_pts


def gaussian_2d(y, x, a, b, p, u, v):
    return (np.exp(((a * v) ** 2 + (b * u) ** 2 + (a * x * np.cos(y)) ** 2
                    + (b * x * np.sin(y)) ** 2 - 2 * v * x * np.cos(y) * a ** 2
                    - 2 * u * x * np.sin(y) * b ** 2 - 2 * a * b * p * u * v
                    - a * b * p * np.sin(
            2 * y) * x ** 2 + 2 * a * b * p * u * x * np.cos(y)
                    + 2 * a * b * p * v * x * np.sin(y)) / (
                           2 * (p ** 2 - 1) * (a * b) ** 2)) * x
            ) / (2 * np.pi * a * b * np.sqrt(1 - p ** 2))


def gaussian_2d2(y, x, cov, u, v):
    X = np.asarray([[x - u, y - v]])
    return 1 / (2 * np.pi * np.sqrt(det(cov))) * \
           np.exp(-0.5 * np.matmul(np.matmul(X, inv(cov)), X.T))


def gaussian_1d(x, mu, sigma):
    return


def color_converter(value):
    if value == -1:
        return 128
    elif value == 0:
        return 255
    else:
        return 0


# def to_grid_locked(pose, offset_x, offset_y, resolution, height):
#     return (int(round((pose[0] - offset_x)/resolution)),
#             height - int(round((pose[1] - offset_y)/resolution)))
#
#
# def to_map_locked(coord, offset_x, offset_y, resolution, height):
#     return (offset_x + resolution*coord[0],
#             offset_y + (height - coord[1])*resolution)

def value_iteration(P, nA, gamma=0.9, tol=1e-8):
    """
    Learn value function and policy by using value iteration method for a given
    gamma and environment.

    Parameters:
    ----------
    V: value to be updated
    tol: float
        Terminate value iteration when
            max |value_function(s) - prev_value_function(s)| < tol
    Returns:
    ----------
    policy_new: np.ndarray[nS,nA]
    V_new: np.ndarray[nS]
    """

    V_new = {}
    ############################
    # YOUR IMPLEMENTATION HERE #
    delta = np.infty
    while delta >= tol:
        old_value_function = V_new.copy()
        delta_new = 0
        for s in P:
            Q_s = np.zeros(nA)
            for a in range(nA):
                transitions = P[s][a]
                for transition in transitions:
                    probability = transition[0]
                    nextstate = transition[1]
                    reward = transition[2]
                    terminal = transition[3]

                    if terminal:
                        Q_s[a] = Q_s[a] + probability * reward
                    else:
                        Q_s[a] = Q_s[a] + probability * (
                            reward + gamma * V_new.get(nextstate, 0))

            V_new[s] = np.max(Q_s)

            old_value = old_value_function.get(s)
            if old_value is not None:
                delta_new = max(delta_new, np.abs(V_new[s] - old_value))
            else:
                delta_new = max(delta_new, V_new[s])
        delta = delta_new

    policy_new = {}
    for s in P:
        Q_s = np.zeros(nA)
        for a in range(nA):
            transitions = P[s][a]
            for transition in transitions:
                probability = transition[0]
                nextstate = transition[1]
                reward = transition[2]
                terminal = transition[3]

                if terminal:
                    Q_s[a] = Q_s[a] + probability * reward
                else:
                    Q_s[a] = Q_s[a] + probability * (
                        reward + gamma * V_new[nextstate])

        a_star = np.argmax(Q_s)
        # policy_new[s] = np.zeros(nA)
        # policy_new[s][a_star] = 1
        policy_new[s] = a_star

    ############################
    return policy_new, V_new


class MapObject:
    def __init__(self, object_id, pos, pos_var, class_probs):
        self.id = object_id
        self.pos = pos
        self.pos_var = pos_var
        self.class_probs = class_probs
        self.visualized = False
        # self.grip_pub = rospy.Publisher("/grip_image_" + str(object_id), Image, queue_size=10)

    def update(self, pos, pos_var, class_probs):
        self.pos = pos
        self.pos_var = pos_var
        self.class_probs = class_probs


class Planner:
    def __init__(self):
        self.target_frame = "camera_link"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.ang_var = None
        self.pos_var = None

        self.old_ang = None
        self.ang = None
        self.pos = None
        self.vel = None

        self.sigma_p = None
        self.sigma_p_inv = None

        self.grid_map_grey = None
        self.seg_map = None
        self.old_seg_map = None
        self.old_grid_map = None
        self.grid_map_color = None
        self.obstacle_map = None

        self.goal_map = None
        self.goal_phase = None
        self.goal_prob = None

        self.frontier_phase = None
        self.frontier_map = None
        self.frontier_prob = None
        self.frontier_centroid = None
        self.frontier_num_labels = None
        self.frontier_stat = None

        self.unknown_map = None
        self.value_image = None

        self.width = None
        self.height = None

        self.old_height = None
        self.old_width = None

        self.resolution = None
        self.old_resolution = None

        self.offset_x = None
        self.offset_y = None

        self.old_offset_x = None
        self.old_offset_y = None

        self.mc = None
        self.mr = None

        self.grip_success = {}
        self.objects = {}
        self.sightings_map = {}
        self.sightings_phase = {}
        self.sightings_prob = {}
        self.room_ids_2_objects = {}
        self.room_ids_2_prob = {}

        self.bridge = CvBridge()

        self.seq = 1

        self.goal = None
        self.path = set()
        self.waypoint = None
        self.goal_status = False
        self.ang_goal1_status = False
        self.ang_goal2_status = False
        self.ang_calculation = True

        self.ang_goal1 = None
        self.ang_goal2 = None

        self.initial_obs = True
        self.in_obstacle = False
        self.replan = True
        self.inherit_heuristic = False
        self.observed_ang = 0

        self.blind_radius = 1.0
        self.action_deltas = [[1, -1],
                              [1, 0],
                              [1, 1],
                              [0, 1],
                              [-1, 1],
                              [-1, 0],
                              [-1, -1],
                              [0, -1]]
        self.nA = 8

        self.action_probabilities = (0.8, 0.1, 0.1)

        self.V = {}
        self.gamma = 0.9
        self.P = {}
        self.policy = None

        self.classes = ['towel', 'objects', 'lighting', 'stool', 'counter',
                        'door', 'clothes', 'appliances', 'furniture',
                        'shelving', 'bed', 'blinds', 'table', 'cabinet',
                        'shower', 'chair', 'chest_of_drawers', 'tv_monitor',
                        'toilet', 'mirror', 'sofa', 'cushion', 'sink',
                        'banana', 'cheezit']

        self.evidence_classes = ['toilet', 'lighting', 'sink', 'mirror',
                                 'shower']

        self.agent_radius = 0.15
        self.angular_error_threshold = 0.5
        self.max_linear_speed = 0.2
        self.max_turn_speed = 0.2
        mat = io.loadmat(
            'pos.mat')
        self.circle_pos = mat['pos']
        self.circle_op = mat['op']
        self.max_error = 0

        self.mutex = Lock()
        self.value_mutex = Lock()

        self.map_client = actionlib.SimpleActionClient(
            '/room_segmentation/room_segmentation_server',
            MapSegmentationAction)
        rospy.loginfo("Waiting for action server to start.")
        # wait for the action server to start
        self.map_client.wait_for_server()  # will wait for infinite time
        rospy.loginfo("Action server started, sending goal.")

        self.odom_sub = rospy.Subscriber("/rtabmap/odom", Odometry,
                                         self.odom_callback)
        self.semantic_map_sub = rospy.Subscriber("/semantic_map",
                                                 Object2DArray,
                                                 self.semantic_map_callback)
        self.map_sub = rospy.Subscriber("/rtabmap/grid_map", OccupancyGrid,
                                        self.map_callback)

        self.map_click_sub = rospy.Subscriber("/map_image_mouse_left", Point,
                                              self.map_click_callback)
        self.image_pub = rospy.Publisher("/map_image", Image, queue_size=10)
        self.debug_pub = rospy.Publisher("/debug_image", Image, queue_size=10)
        self.object_info_pub = rospy.Publisher("/obj_info", Image,
                                               queue_size=10)
        self.command_pub = rospy.Publisher("/robot_command", RobotCommand,
                                           queue_size=10)
        self.obstacle_map_pub = rospy.Publisher("/obstacle_map", Image,
                                                queue_size=10)
        self.frontier_map_pub = rospy.Publisher("/frontier_map", Image,
                                                queue_size=10)
        self.seg_map_pub = rospy.Publisher("/seg_map", Image, queue_size=10)
        self.sightings_map_pub = rospy.Publisher("/sighting_map",
                                                 Image, queue_size=10)
        self.c = rospy.ServiceProxy('BBN_infer', BBNInfer)
        rospy.wait_for_service('BBN_infer')
        self.rate = rospy.Rate(20)  # 3hz
        self.dt = 1 / 30.0
        self.iteration = 0

    def select_action(self, state, train=False):
        Q_s = np.zeros(self.nA)
        for a in range(self.nA):
            StateActions = self.P.get(state, False)
            if not StateActions:
                return -1

            transitions = StateActions[a]
            for transition in transitions:
                probability = transition[0]
                nextstate = transition[1]
                reward = transition[2]
                terminal = transition[3]

                if terminal:
                    Q_s[a] = Q_s[a] + probability * reward
                else:
                    Q_s[a] = Q_s[a] + probability * (
                        reward + self.gamma * self.V[nextstate])

        # select an action
        if train:
            action_probs = softmax(Q_s)
            action = np.random.choice(self.nA, p=action_probs)
            self.value_mutex.acquire()
            self.V[state] = np.max(Q_s)
            self.value_mutex.release()
        else:
            actions = np.flatnonzero(Q_s == np.max(Q_s))
            action = np.random.choice(actions)
        return action

    def Q_value(self, state):
        Q_s = np.zeros(self.nA)
        for a in range(self.nA):
            transitions = self.P[state][a]
            for transition in transitions:
                probability = transition[0]
                nextstate = transition[1]
                reward = transition[2]
                terminal = transition[3]

                if terminal:
                    Q_s[a] = Q_s[a] + probability * reward
                else:
                    Q_s[a] = Q_s[a] + probability * (
                        reward + self.gamma * self.V[nextstate])

        return Q_s

    def trial(self, s0):
        done = False
        state = s0
        states = []
        while not done:
            action = self.select_action(state, train=True)
            # return a reward and new state
            transitions = self.P[state][action]
            probabilities, nextstates, rewards, terminals = zip(*transitions)

            if np.min(probabilities) < 0:
                probabilities = np.asarray(probabilities) - np.min(
                    probabilities)
            probabilities = probabilities / np.sum(probabilities)

            index = np.random.choice(len(nextstates), p=probabilities)

            new_state = nextstates[index]
            done = terminals[index]

            # append state, action, reward to episode
            states.append(state)
            # update state to new state
            state = new_state

            if len(states) > 10 * (self.old_width + self.old_height):
                break
        return states

    def backup(self, states):
        states.reverse()
        for state in states:
            Q_s = np.zeros(self.nA)
            for a in range(self.nA):
                transitions = self.P[state][a]
                for transition in transitions:
                    probability = transition[0]
                    nextstate = transition[1]
                    reward = transition[2]
                    terminal = transition[3]

                    if terminal:
                        Q_s[a] = Q_s[a] + probability * reward
                    else:
                        Q_s[a] = Q_s[a] + probability * (
                            reward + self.gamma * self.V.get(nextstate, 0))

            self.V[state] = np.max(Q_s)

    def run(self):
        while not rospy.is_shutdown():
            if self.obstacle_map is not None and not self.initial_obs:

                # grid_offset_x = 0
                # grid_offset_y = 0
                # frontier_new = None

                pos_grid = np.asarray(self.to_grid(self.pos))
                if self.grid_map_grey[pos_grid[1], pos_grid[0]] != 255:
                    self.replan = True
                    continue

                if self.replan:
                    self.iteration = 0
                    self.mutex.acquire()

                    # if self.old_resolution == self.resolution:
                    #     self.inherit_heuristic = True
                    #     grid_offset_x = int(round((self.old_offset_x -
                    #                                self.offset_x)/self.resolution))
                    #
                    #     grid_offset_y = int(round(self.height - self.old_height +
                    #                               (self.offset_y - self.old_offset_y)/self.resolution))
                    #
                    #     old_frontier_map = np.zeros_like(self.frontier_map)
                    #
                    #     height = min(self.old_height, self.height,
                    #                  self.height-grid_offset_y)
                    #     width = min(self.old_width, self.width,
                    #                 self.width - grid_offset_x)
                    #
                    #     if grid_offset_x >= 0 and grid_offset_y >= 0:
                    #         old_frontier_map[
                    #             grid_offset_y:(height + grid_offset_y),
                    #             grid_offset_x:(width + grid_offset_x)] \
                    #             = self.old_frontier_map[0:height, 0:width]
                    #     elif grid_offset_x >= 0 and grid_offset_y < 0:
                    #         old_frontier_map[0:(height + grid_offset_y),
                    #                          grid_offset_x:(width + grid_offset_x)] \
                    #             = self.old_frontier_map[-grid_offset_y:height,
                    #                                     0:width]
                    #     elif grid_offset_x < 0 and grid_offset_y >= 0:
                    #         old_frontier_map[
                    #             grid_offset_y:(height + grid_offset_y),
                    #             0:(width + grid_offset_x)] \
                    #             = self.old_frontier_map[0:height,
                    #                                     -grid_offset_x:width]
                    #     else:
                    #         old_frontier_map[
                    #             0:(height + grid_offset_y),
                    #             0:(width + grid_offset_x)] \
                    #             = self.old_frontier_map[-grid_offset_y:height,
                    #                                     -grid_offset_x:width]
                    #
                    #     frontier_new = np.logical_and(self.frontier_map,
                    #                                   np.logical_not(
                    #                                       old_frontier_map)
                    #                                   ).astype(float)

                    self.old_grid_map = self.grid_map_grey.copy()
                    self.old_seg_map = self.seg_map.copy()
                    obstacle_map = self.obstacle_map.copy()
                    for obj_id in self.sightings_map:
                        self.goal_map = self.sightings_map[obj_id].copy()
                        self.goal_prob = self.sightings_prob[obj_id].copy()
                        self.goal_phase = self.sightings_phase[obj_id].copy()

                    # self.goal_map = self.frontier_map.copy()
                    # self.goal_prob = self.frontier_prob.copy()
                    # self.goal_phase = self.frontier_phase.copy()

                    self.old_height = self.height
                    self.old_width = self.width
                    self.old_resolution = self.resolution
                    self.old_offset_x = self.offset_x
                    self.old_offset_y = self.offset_y

                    self.mutex.release()

                    # for obj_id, obj in self.objects.items():
                    #     distance_cov = obj.pos_var + np.diag([self.pos_var, self.pos_var])
                    #     cov_a = np.sqrt(obj.pos_var[0, 0] + self.pos_var)
                    #     cov_b = np.sqrt(obj.pos_var[1, 1] + self.pos_var)
                    #     p = obj.pos_var[0, 1] / (cov_a * cov_b)
                    #     grip_array = np.zeros((self.old_height, self.old_width))
                    #
                    #     center_coordinates = self.to_grid_old(obj.pos)
                    #
                    #     xmin = int(math.floor(max(
                    #         center_coordinates[0] - 3 * cov_a/self.old_resolution - 2, 0)))
                    #     xmax = int(math.ceil(min(
                    #         center_coordinates[0] + 3 * cov_a/self.old_resolution + 3, self.old_width)))
                    #
                    #     ymin = int(math.floor(max(
                    #         center_coordinates[1] - 3 * cov_b/self.old_resolution - 2, 0)))
                    #     ymax = int(math.ceil(min(
                    #         center_coordinates[1] + 3 * cov_b/self.old_resolution + 3, self.old_height)))
                    #
                    #     # start_time = time()
                    #     for i in range(xmin, xmax):
                    #         for j in range(ymin, ymax):
                    #             u, v = obj.pos - self.to_map_old([i, j])
                    #             # grip_success, tol = dblquad(func=gaussian_2d, a=0.0,
                    #             #                             b=0.1,
                    #             #                             gfun=lambda x: -np.pi,
                    #             #                             hfun=lambda x: np.pi,
                    #             #                             args=(cov_a, cov_b, p, u, v))
                    #             #
                    #             cdf = mvn.cdf(self.circle_pos, mean=[u, v], cov=distance_cov)
                    #             grip_success = np.matmul(self.circle_op, cdf)[0]
                    #             grip_array[j, i] = grip_success
                    #             # self.max_error = max(self.max_error, np.abs(grip_success3 - grip_success))
                    #             # print('max error is ', self.max_error)
                    #     # end_time = time()
                    #     self.grip_success[obj_id] = grip_array
                    #     # print('time passed is ', start_time - end_time)
                    #     image_message = self.bridge.cv2_to_imgmsg(
                    #         (grip_array*255).astype(np.uint8), encoding="mono8")
                    #
                    #     obj.grip_pub.publish(image_message)

                    free_space_grid = (self.old_grid_map == 255).astype(
                        np.uint8)
                    kernel = np.ones((3, 3), np.uint8)
                    state_space_grid = cv2.dilate(free_space_grid, kernel,
                                                  iterations=1,
                                                  borderType=cv2.BORDER_ISOLATED)

                    states = np.nonzero(state_space_grid)
                    states_index = np.ravel_multi_index(states, (
                    self.old_height, self.old_width))
                    states = np.asarray(states).T
                    self.P = {}

                    Vh = self.goal_map.copy()
                    count = 0
                    while Vh[pos_grid[1], pos_grid[0]] == 0:
                        Vh_new = cv2.dilate(Vh, kernel, iterations=1,
                                            borderType=cv2.BORDER_ISOLATED) * self.gamma
                        Vh = np.maximum(Vh, Vh_new)
                        count += 1

                    Vh[Vh == 0] = np.amax(self.goal_map) * np.power(self.gamma,
                                                                    count)

                    # Vh = -np.ones((self.old_height, self.old_width))
                    # count = 0
                    # Vh[self.goal_map.astype(bool)] = 0
                    # frontier_map = self.goal_map.copy()
                    # while Vh[pos_grid[1], pos_grid[0]] < 0:
                    #     frontier_map_new = cv2.dilate(frontier_map, kernel,
                    #                                   iterations=1,
                    #                                   borderType=cv2.BORDER_ISOLATED)
                    #     frontier_diff = numpy.logical_xor(frontier_map_new,
                    #                                       frontier_map)
                    #
                    #     Vh[frontier_diff] = count
                    #     count += 1
                    #     frontier_map = frontier_map_new
                    #
                    # Vh[Vh < 0] = count
                    # Vh = np.power(self.gamma, Vh)

                    # self.inherit_heuristic = False
                    # if self.inherit_heuristic:
                    #     frontier_map = frontier_new
                    #     Vh2 = -np.ones_like(self.old_frontier_map)
                    #     count = 0
                    #     while Vh2[pos_grid[1], pos_grid[0]] < 0:
                    #         frontier_map_new = cv2.dilate(frontier_map, kernel,
                    #                                       iterations=1,
                    #                                       borderType=cv2.BORDER_ISOLATED)
                    #         frontier_diff = numpy.logical_xor(frontier_map_new,
                    #                                           frontier_map)
                    #
                    #         Vh2[frontier_diff] = count
                    #         count += 1
                    #         frontier_map = frontier_map_new
                    #
                    #     Vh2[Vh2 < 0] = count
                    #     Vh2 = np.power(self.gamma, Vh2)
                    #
                    #     old_value = np.zeros_like(self.old_frontier_map)
                    #
                    #     height = min(self.old_height, self.value_image.shape[0],
                    #                  self.value_image.shape[0]-grid_offset_y)
                    #     width = min(self.old_width, self.value_image.shape[1],
                    #                 self.value_image.shape[1] - grid_offset_x)
                    #
                    #     if grid_offset_x >= 0 and grid_offset_y >= 0:
                    #         old_value[grid_offset_y:(height + grid_offset_y),
                    #                   grid_offset_x:(width + grid_offset_x)] \
                    #             = self.value_image[0:height, 0:width]
                    #     elif grid_offset_x >= 0 and grid_offset_y < 0:
                    #         old_value[0:(height + grid_offset_y),
                    #                   grid_offset_x:(width + grid_offset_x)] \
                    #             = self.value_image[-grid_offset_y:height,
                    #                                0:width]
                    #     elif grid_offset_x < 0 and grid_offset_y >= 0:
                    #         old_value[grid_offset_y:(height + grid_offset_y),
                    #                   0:(width + grid_offset_x)] \
                    #             = self.value_image[0:height,
                    #                                -grid_offset_x:width]
                    #     else:
                    #         old_value[0:(height + grid_offset_y),
                    #                   0:(width + grid_offset_x)] \
                    #             = self.value_image[-grid_offset_y:height,
                    #                                -grid_offset_x:width]
                    #
                    #     Vh2 = np.maximum(Vh2, old_value)
                    #     Vh = np.minimum(Vh2, Vh)

                    # image_message = self.bridge.cv2_to_imgmsg(
                    #     (Vh*255).astype(np.uint8), encoding="mono8")
                    #
                    # self.debug_pub.publish(image_message)

                    for j in range(states.shape[0]):
                        state_index = states_index[j]
                        state = states[j]
                        self.P[state_index] = {}
                        self.V[state_index] = Vh[state[0], state[1]]
                        for action in range(self.nA):
                            action_deltas = np.take(self.action_deltas,
                                                    [action, action + 1,
                                                     action - 1],
                                                    axis=0, mode='wrap')

                            next_states = state + action_deltas
                            next_states[next_states[:, 0] < 0, 0] = 0
                            next_states[next_states[:, 1] < 0, 1] = 0
                            next_states[next_states[:,
                                        0] > self.old_height - 1, 0] = self.old_height - 1
                            next_states[next_states[:,
                                        1] > self.old_width - 1, 1] = self.old_width - 1

                            next_states_frontier = np.hstack((np.ones(
                                (3, 1), dtype=np.int8), next_states))

                            next_states_obstacle = np.hstack((2 * np.ones(
                                (3, 1), dtype=np.int8), next_states))

                            next_states_indices = np.ravel_multi_index(
                                next_states.T,
                                (self.old_height,
                                 self.old_width))

                            out_of_free_grids = np.in1d(next_states_indices,
                                                        states_index,
                                                        invert=True)

                            next_frontier_states = np.ravel_multi_index(
                                next_states_frontier.T,
                                (3, self.old_height, self.old_width))

                            next_obstacle_states = np.ravel_multi_index(
                                next_states_obstacle.T,
                                (3, self.old_height, self.old_width))

                            pnsrt = []

                            for i in range(3):
                                next_state = next_states[i]
                                pnsrt.append(((1 - self.goal_prob[
                                    next_state[0], next_state[1]]
                                               - obstacle_map[
                                                   next_state[0], next_state[
                                                       1]]) *
                                              self.action_probabilities[i],
                                              next_states_indices[i], 0,
                                              out_of_free_grids[i]))

                                if self.goal_prob[
                                    next_state[0], next_state[1]] > 0:
                                    pnsrt.append((self.goal_prob[next_state[0],
                                                                 next_state[
                                                                     1]] *
                                                  self.action_probabilities[i],
                                                  next_frontier_states[i],
                                                  self.goal_map[next_state[0],
                                                                next_state[1]],
                                                  True))

                                if obstacle_map[
                                    next_state[0], next_state[1]] > 0:
                                    pnsrt.append((obstacle_map[next_state[0],
                                                               next_state[1]] *
                                                  self.action_probabilities[i],
                                                  next_obstacle_states[i], 0,
                                                  True))

                            self.P[state_index][action] = pnsrt
                    self.replan = False

                start_time = time()

                pos_index = np.ravel_multi_index(np.flipud(pos_grid),
                                                 (self.old_height,
                                                  self.old_width))

                if pos_index not in self.P.keys():
                    self.replan = True
                    continue

                states = self.trial(pos_index)
                # old_states = []
                # loop = False
                # loop_states = []
                # if len(states) > 2 * (self.old_width + self.old_height):
                #     for state in states:
                #         if state in old_states:
                #             loop = True
                #         old_states.append(state)
                #         if loop:
                #             loop_states.append(state)
                #
                #
                #     V_debug = np.zeros_like(self.old_frontier_map)
                #     for state_index in loop_states:
                #         state = np.unravel_index(state_index,
                #                                  (self.old_height, self.old_width))
                #         V_debug[state[0], state[1]] = self.V.get(state_index, 0)
                #
                #         for action_delta in self.action_deltas:
                #             neighbor = state + np.asarray(action_delta)
                #             neighbor_index = np.ravel_multi_index(neighbor, (
                #                 self.old_height, self.old_width))
                #             V_debug[neighbor[0], neighbor[1]] = self.V.get(
                #                 neighbor_index, 0)
                #
                #     nonzero_indices = np.transpose(np.nonzero(V_debug))
                #     xy_max = np.amax(nonzero_indices, axis=0)
                #     xy_min = np.amin(nonzero_indices, axis=0)
                #
                #     nonzero_values = V_debug[xy_min[0]-1: xy_max[0]+1,
                #                              xy_min[1]-1: xy_max[1]+1]
                #
                #     value_grey = cv2.normalize(src=nonzero_values, dst=None,
                #                                alpha=0, beta=255,
                #                                norm_type=cv2.NORM_MINMAX,
                #                                dtype=cv2.CV_8UC1)
                #
                #     image_path = '/home/zhentian/Pictures/'
                #     image_name = datetime.now().strftime('%Y-%m-%d %H-%M-%S') + '.jpg'
                #
                #     cv2.imwrite(image_path + image_name, value_grey)

                # self.value_mutex.acquire()
                # self.backup(states)
                # self.value_mutex.release()
                self.iteration += 1

                # self.policy, self.V = value_iteration(P, self.nA)
                self.value_image = np.zeros((self.old_height, self.old_width))
                for state_index in self.V:
                    if state_index > (self.old_width * self.old_height - 1):
                        continue
                    state = np.unravel_index(state_index,
                                             (self.old_height, self.old_width))
                    self.value_image[state[0], state[1]] = self.V[state_index]

                # self.value_image = 255/np.amax(self.value_image)*self.value_image
                # self.value_image = np.clip(self.value_image, 0, 255).astype(np.uint8)
                debug_image_color = cv2.cvtColor(self.old_grid_map,
                                                 cv2.COLOR_GRAY2BGR)

                # path_stop = False
                # pos_grid = np.asarray(self.to_grid_old(self.initial_pos)).\
                #     astype(np.uint8)
                # while not path_stop:
                #     value = self.value_image[pos_grid[1], pos_grid[0]]
                #     path_stop = True
                #     debug_image_color[pos_grid[1], pos_grid[0], :] = [48, 172, 119]
                #     max_value_grid = pos_grid
                #     for action_delta in self.action_deltas:
                #         next_pos_grid = pos_grid + np.asarray(action_delta).\
                #             astype(np.uint8)
                #         if self.value_image[next_pos_grid[1], next_pos_grid[0]] > value:
                #             value = self.value_image[next_pos_grid[1], next_pos_grid[0]]
                #             max_value_grid = next_pos_grid
                #             path_stop = False
                #     pos_grid = max_value_grid

                path = set()
                while True:
                    debug_image_color[pos_grid[1], pos_grid[0], :] = [189, 114,
                                                                      0]
                    if tuple(pos_grid) in path:
                        break
                    else:
                        path.add(tuple(pos_grid))
                    pos_index = np.ravel_multi_index(np.flipud(pos_grid),
                                                     (self.old_height,
                                                      self.old_width))
                    # if self.policy.get(pos_index, -1) < 0:
                    #     print('this grid has no policy')
                    #     break
                    # else:
                    # action_index = self.policy[pos_index]
                    action_index = self.select_action(pos_index)
                    pos_grid = pos_grid + np.flipud(
                        self.action_deltas[action_index])

                # for state_index in states:
                #     state = np.unravel_index(state_index,
                #                              (self.old_height, self.old_width))
                #     debug_image_color[state[0], state[1], :] = [189, 114, 0]

                image_message = self.bridge.cv2_to_imgmsg(debug_image_color,
                                                          encoding="bgr8")
                self.debug_pub.publish(image_message)
                end_time = time()
                print('time spent in iteration is ', end_time - start_time)

            # self.rate.sleep()

    def angle_tracking(self, angle_error):
        rot_dir = 1.0
        if angle_error < 0:
            rot_dir = -1.0

        angular_correction = 0.0
        if np.abs(angle_error) > (
            self.max_turn_speed * 10.0 * self.dt):
            angular_correction = self.max_turn_speed
        else:
            angular_correction = np.abs(angle_error) / 2.0

        angular_correction = np.clip(rot_dir * angular_correction,
                                     -self.max_turn_speed,
                                     self.max_turn_speed)

        return angular_correction

    def odom_callback(self, msg):
        # only takes the covariance, the pose is taken from tf transformation
        self.pos_var = msg.pose.covariance[0]
        self.ang_var = msg.pose.covariance[-1]

        self.sigma_p = np.diag([self.pos_var, self.pos_var, self.ang_var])
        self.sigma_p_inv = inv(self.sigma_p)

        self.vel = msg.twist.twist.linear.x

        from_frame_rel = self.target_frame
        to_frame_rel = 'map'

        try:
            trans = self.tfBuffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rospy.Time(0))

            self.pos = np.asarray([trans.transform.translation.x,
                                   trans.transform.translation.y])

            self.old_ang = self.ang
            self.ang = tf.transformations.euler_from_quaternion(
                [trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w])[-1]

        except (TransformException, ConnectivityException,
                ExtrapolationException) as ex:
            rospy.loginfo('Could not transform %s to %s: ', to_frame_rel,
                          from_frame_rel)
            return

        if self.initial_obs and self.old_ang is not None:
            self.observed_ang += np.arctan2(
                np.sin(self.ang - self.old_ang),
                np.cos(self.ang - self.old_ang))

            if self.observed_ang > 2 * np.pi:
                self.initial_obs = False

        command_msg = RobotCommand()
        if self.initial_obs:
            command_msg.linear_vel = 0
            command_msg.angular_vel = 0.3
        else:
            pos_grid = self.to_grid(self.pos)
            if self.grid_map_grey[pos_grid[1], pos_grid[0]] != 255:
                free_pnts = np.asarray(np.where(self.grid_map_grey == 255)).T
                distances = norm(free_pnts - np.flipud(pos_grid), axis=1)
                goal_grid = free_pnts[np.argmin(distances), :]

                self.goal = self.to_map(np.flipud(goal_grid))
                to_waypoint = self.goal - self.pos
                angle_error = np.arctan2(to_waypoint[1],
                                         to_waypoint[0]) - self.ang
                angle_error = np.arctan2(np.sin(angle_error),
                                         np.cos(angle_error))

                if np.abs(angle_error) < self.angular_error_threshold:
                    # speed up to max
                    # the positive direction of car bearing is opposite in
                    # simulation
                    new_velocity = (-self.vel - self.max_linear_speed) / 2.0
                else:
                    # slow down to 0
                    new_velocity = -self.vel / 2.0

                command_msg.linear_vel = new_velocity
                command_msg.angular_vel = self.angle_tracking(angle_error)

            elif self.iteration > 300 and not self.goal_status and not self.replan:
                pos_grid = self.to_grid_old(self.pos)
                pos_index = np.ravel_multi_index(np.flipud(pos_grid),
                                                 (self.old_height,
                                                  self.old_width))
                self.goal_status = self.goal_map[pos_grid[1], pos_grid[0]] > 0
                # action_index = self.policy[pos_index]
                if self.old_grid_map[pos_grid[1], pos_grid[0]] == 255:
                    self.value_mutex.acquire()
                    action_index = self.select_action(pos_index)
                    self.value_mutex.release()

                    if action_index >= 0:
                        goal_grid = np.flipud(
                            self.action_deltas[action_index]) + \
                                    np.asarray(pos_grid)

                        # if self.goal is None:
                        self.goal = self.to_map_old(goal_grid)

                # goal = np.asarray([0, 1])

                to_waypoint = self.goal - self.pos
                angle_error = np.arctan2(to_waypoint[1],
                                         to_waypoint[0]) - self.ang
                angle_error = np.arctan2(np.sin(angle_error),
                                         np.cos(angle_error))

                # print('current grid is ', pos_grid)
                # print('goal grid is ', goal_grid)
                # print('current pose is ', self.pos)
                # print('goal pose is ', self.goal)
                # print('actual angle is ', self.ang)
                # print('desired angle is ', np.arctan2(to_waypoint[1], to_waypoint[0]))
                #
                # print('angular error is ', angle_error)

                # print('error is', np.linalg.norm(to_waypoint), angle_error)

                if np.abs(angle_error) < self.angular_error_threshold:
                    # speed up to max
                    # the positive direction of car bearing is opposite in
                    # simulation
                    new_velocity = (-self.vel - self.max_linear_speed) / 2.0
                else:
                    # slow down to 0
                    new_velocity = -self.vel / 2.0

                command_msg.linear_vel = new_velocity
                command_msg.angular_vel = self.angle_tracking(angle_error)
            else:
                if self.goal_status:
                    if self.ang_calculation:
                        pos_grid = self.to_grid_old(self.pos)
                        angle_error1 = self.goal_phase[pos_grid[1],
                                                       pos_grid[0]] \
                                       - self.ang + 0.3 * np.pi

                        angle_error2 = self.goal_phase[pos_grid[1],
                                                       pos_grid[0]] \
                                       - self.ang - 0.3 * np.pi

                        angle_error1 = np.arctan2(np.sin(angle_error1),
                                                  np.cos(angle_error1))

                        angle_error2 = np.arctan2(np.sin(angle_error2),
                                                  np.cos(angle_error2))

                        if np.abs(angle_error1) < np.abs(angle_error2):
                            self.ang_goal1 = self.goal_phase[pos_grid[1],
                                                             pos_grid[0]] \
                                             + 0.3 * np.pi

                            self.ang_goal2 = self.goal_phase[pos_grid[1],
                                                             pos_grid[0]] \
                                             - 0.3 * np.pi
                        else:
                            self.ang_goal1 = self.goal_phase[pos_grid[1],
                                                             pos_grid[0]] \
                                             - 0.3 * np.pi

                            self.ang_goal2 = self.goal_phase[pos_grid[1],
                                                             pos_grid[0]] \
                                             + 0.3 * np.pi

                    self.ang_calculation = False

                    if not self.ang_goal1_status:
                        angle_error = self.ang_goal1 - self.ang
                        angle_error = np.arctan2(np.sin(angle_error),
                                                 np.cos(angle_error))

                        command_msg.linear_vel = 0
                        command_msg.angular_vel = self.angle_tracking(
                            angle_error)

                        if np.abs(angle_error) < 0.1:
                            self.ang_goal1_status = True

                        print('tracking angular goal 1')
                    elif not self.ang_goal2_status:
                        angle_error = self.ang_goal2 - self.ang
                        angle_error = np.arctan2(np.sin(angle_error),
                                                 np.cos(angle_error))

                        command_msg.linear_vel = 0
                        command_msg.angular_vel = self.angle_tracking(
                            angle_error)

                        if np.abs(angle_error) < 0.1:
                            self.ang_goal2_status = True

                        print('tracking angular goal 2')
                    else:
                        self.replan = True
                        self.goal_status = False

                        self.ang_goal1_status = False
                        self.ang_goal2_status = False
                        self.ang_calculation = True

                        command_msg.linear_vel = 0
                        command_msg.angular_vel = 0

        self.command_pub.publish(command_msg)

    def to_grid(self, pose):
        return (int(round((pose[0] - self.offset_x) / self.resolution)),
                self.height - int(
                    round((pose[1] - self.offset_y) / self.resolution)))

    def to_map(self, coord):
        return (self.offset_x + self.resolution * coord[0],
                self.offset_y + (self.height - coord[1]) * self.resolution)
        # return self.mc(pose[0]), self.mr(pose[1])

    def to_grid_old(self, pose):
        return (
        int(round((pose[0] - self.old_offset_x) / self.old_resolution)),
        self.old_height - int(
            round((pose[1] - self.old_offset_y) / self.old_resolution)))

    def to_map_old(self, coord):
        return (self.old_offset_x + self.old_resolution * coord[0],
                self.old_offset_y + (
                        self.old_height - coord[1]) * self.old_resolution)

    def save_data(self):
        pass

    def map_click_callback(self, msg):
        if not self.objects:
            return
        clicked_pt = np.asarray([msg.x, msg.y])
        dists = []
        obj_ids = []
        for obj_id, obj in self.objects.items():
            object_grid = np.asarray(self.to_grid(obj.pos))
            dist = norm(clicked_pt - object_grid)
            obj_ids.append(obj_id)
            dists.append(dist)

        clicked_object = self.objects[obj_ids[np.argmin(dists)]]
        if clicked_object.visualized:
            clicked_object.visualized = False
            print('now is false')
        else:
            clicked_object.visualized = True
            print('now is true')

    def map_callback(self, msg):
        """Callback from map
        :type msg: OccupancyGrid
        """
        np_data = np.array([color_converter(e) for e in msg.data])
        room_data = np_data.copy().astype(np.uint8)
        room_data[room_data == 128] = 0
        room_data = np.flipud(np.reshape(room_data, (msg.info.height,
                                                     msg.info.width)))
        goal = MapSegmentationGoal()
        goal.map_origin.position.x = msg.info.origin.position.x
        goal.map_origin.position.y = msg.info.origin.position.y
        goal.map_resolution = msg.info.resolution
        goal.return_format_in_meter = False
        goal.return_format_in_pixel = True
        goal.robot_radius = self.agent_radius

        if self.mutex.acquire(False):
            self.resolution = msg.info.resolution

            self.height = msg.info.height
            self.width = msg.info.width

            self.offset_x = msg.info.origin.position.x
            self.offset_y = msg.info.origin.position.y

            # self.mc = interp1d([self.offset_x, self.offset_x + self.width * self.resolution],
            #                    [0, self.width], bounds_error=False)
            # self.mr = interp1d([self.offset_y, self.offset_y + self.height * self.resolution],
            #                    [self.height, 0], bounds_error=False)

            self.grid_map_grey = np.flipud(np.reshape(np_data,
                                                      (self.height, self.width)
                                                      )).astype('uint8')

            if self.pos is not None:
                roi = np.zeros_like(self.grid_map_grey)
                cv2.circle(roi, self.to_grid(self.pos),
                           int(self.blind_radius / self.resolution), 1, -1)

                mask = np.logical_and(self.grid_map_grey == 128,
                                      roi.astype(bool))

                self.grid_map_grey[mask] = 255

                room_data[mask] = 255
                goal.input_map = self.bridge.cv2_to_imgmsg(room_data,
                                                           encoding="mono8")
                # self.debug_pub.publish(goal.input_map)
                self.map_client.send_goal(goal)
            self.grid_map_color = cv2.cvtColor(self.grid_map_grey,
                                               cv2.COLOR_GRAY2BGR)

            if self.pos is not None:
                self.sightings_map = {}
                self.find_sightings()
                print('completed find sighting')

            self.findFrontier()

            if self.pos is not None:
                sigma_x = np.sqrt(self.pos_var) / self.resolution
                kx = int(math.ceil(
                    max(3 * np.sqrt(self.pos_var) / self.resolution, 1)))
                kx = 2 * kx + 1

                self.obstacle_map = (np.not_equal(self.grid_map_grey, 255)
                                     ).astype(float)

                self.obstacle_map = cv2.GaussianBlur(self.obstacle_map,
                                                     (kx, kx), sigma_x,
                                                     borderType=cv2.BORDER_ISOLATED)

                self.frontier_prob = cv2.GaussianBlur(
                    (self.frontier_map > 0).astype(float),
                    (kx, kx), sigma_x,
                    borderType=cv2.BORDER_ISOLATED)

                for obj_id in self.sightings_map:
                    self.sightings_prob[obj_id] = \
                        cv2.GaussianBlur(self.sightings_map[obj_id], (kx, kx),
                                         sigma_x,
                                         borderType=cv2.BORDER_ISOLATED)

                image_message = self.bridge.cv2_to_imgmsg(
                    (self.obstacle_map * 255).astype(np.uint8),
                    encoding="mono8")
                self.obstacle_map_pub.publish(image_message)

            start_time = time()
            if self.pos is not None and self.map_client.wait_for_result():
                result_seg = self.map_client.get_result()
                result_seg_img = result_seg.segmented_map
                self.seg_map = self.bridge.imgmsg_to_cv2(result_seg_img,
                                                         desired_encoding="passthrough")

                seg_map = self.seg_map.copy()
                seg_map[self.seg_map == 65280] = 0
                seg_map = seg_map * 255 / np.max(seg_map)
                image_message3 = self.bridge.cv2_to_imgmsg(
                    seg_map.astype(np.uint8), encoding="mono8")
                self.seg_map_pub.publish(image_message3)

                self.room_ids_2_objects = {}
                free_pnts = np.asarray(np.where(self.grid_map_grey == 255)).T
                for obj_id, obj in self.objects.items():
                    object_grid = self.to_grid(obj.pos)
                    if object_grid[1] >= self.height:
                        continue

                    if object_grid[1] < 0:
                        continue

                    if object_grid[0] < 0:
                        continue

                    if object_grid[0] >= self.width:
                        continue

                    if self.grid_map_grey[object_grid[1], object_grid[0]] == 0:
                        distances = norm(free_pnts - np.flipud(object_grid),
                                         axis=1)
                        object_grid = np.flipud(free_pnts[np.argmin(
                            distances), :])

                    room_id = self.seg_map[object_grid[1], object_grid[0]]
                    if room_id and room_id != 65280:
                        if room_id not in self.room_ids_2_objects.keys():
                            self.room_ids_2_objects[room_id] = [obj_id]
                        else:
                            self.room_ids_2_objects[room_id].append(obj_id)

                for room_id in self.room_ids_2_objects:
                    evidence = np.zeros(5, dtype='int')
                    objects = self.room_ids_2_objects[room_id]
                    for obj_id in objects:
                        class_prob = self.objects[obj_id].class_probs
                        if class_prob[0] > 0.5:
                            evidence[0] = 1

                        if class_prob[2] > 0.5:
                            evidence[1] = 1

                        if class_prob[22] > 0.5:
                            evidence[2] = 1

                        if class_prob[19] > 0.5:
                            evidence[3] = 1

                        if class_prob[14] > 0.5:
                            evidence[4] = 1

                    req = self.c(evidence.tolist())
                    self.room_ids_2_prob[room_id] = req.p

                valid_pnts = np.asarray(np.where(np.logical_and(
                    self.seg_map > 0, self.seg_map != 65280))).T

                for i in range(1, self.frontier_num_labels):
                    stat = self.frontier_stat[i]
                    area = stat[cv2.CC_STAT_AREA]
                    indices = (self.frontier_map == i)
                    center = self.frontier_centroid[i].astype('int')

                    room_id = self.seg_map[center[1], center[0]]

                    if room_id == 0 or room_id == 65280:
                        distances = norm(valid_pnts - np.flipud(center),
                                         axis=1)
                        center = np.flipud(valid_pnts[np.argmin(distances), :])
                        room_id = self.seg_map[center[1], center[0]]

                    p = self.room_ids_2_prob[room_id]
                    if area > 15:
                        self.frontier_map[indices] = area * p
                end_time = time()

                image_message2 = self.bridge.cv2_to_imgmsg(
                    (self.frontier_map * 255 / np.amax(self.frontier_map)
                     ).astype(np.uint8), encoding="mono8")
                self.frontier_map_pub.publish(image_message2)

                print('time spent waiting is', end_time - start_time)

            self.mutex.release()

    def find_sightings(self):

        max_line_len = 3 / self.resolution
        angles = np.arange(
            -np.pi, np.pi, step=1.0 / max_line_len, dtype=np.float32
        )

        obj_of_interest = None
        max_class_prob = 0
        target_class_index = self.classes.index("towel")

        for obj_id, obj in self.objects.items():
            object_grid = self.to_grid(obj.pos)

            if object_grid[1] >= self.height:
                continue

            if object_grid[1] < 0:
                continue

            if object_grid[0] < 0:
                continue

            if object_grid[0] >= self.width:
                continue

            if self.grid_map_grey[object_grid[1], object_grid[0]] == 128:
                continue

            if obj.class_probs[target_class_index] > max_class_prob:
                max_class_prob = obj.class_probs[target_class_index]
                obj_of_interest = obj_id

        obj = self.objects[obj_of_interest]
        object_grid = self.to_grid(obj.pos)
        start = np.flipud(object_grid).astype(int)
        object_sighting = np.zeros_like(self.grid_map_grey)
        object_phase = np.zeros_like(self.grid_map_grey)
        for angle in angles:
            end = start + max_line_len * np.array([np.cos(angle),
                                                   np.sin(angle)])
            end = np.rint(end).astype(np.int)
            discrete_line = list(zip(*line(start[0], start[1],
                                           end[0], end[1])))

            for pt in discrete_line:
                x, y = pt

                if norm(start - np.asarray(pt), ord=1) < 10:
                    continue

                if x < 0 or x >= self.height:
                    break

                if y < 0 or y >= self.width:
                    break

                if self.grid_map_grey[x, y] != 255:
                    break

                distance = norm(start - np.asarray(pt)) * self.resolution
                if distance > 2:
                    object_sighting[x, y] = 1
                    object_phase[x, y] = -np.arctan2(start[0] - x,
                                                     start[1] - y)

        self.sightings_map[obj_of_interest] = object_sighting
        self.sightings_phase[obj_of_interest] = object_phase

        obj_pose = obj.pos

        center_coordinates = self.to_grid(obj_pose)
        sightings_map = (object_sighting*255).astype('uint8')
        cv2.circle(sightings_map, center_coordinates, 3, 255, -1)
        image_message = self.bridge.cv2_to_imgmsg(sightings_map,
                                                  encoding="mono8")
        self.sightings_map_pub.publish(image_message)


    def findFrontier(self):
        dx = [0, -1, -1, -1, 0, 1, 1, 1]
        dy = [1, 1, 0, -1, -1, -1, 0, 1]

        frontier_mat = cv2.Canny(self.grid_map_grey, 100, 200)

        grad_x = cv2.Scharr(self.grid_map_grey, cv2.CV_32F, 1, 0,
                            borderType=cv2.BORDER_ISOLATED)
        grad_y = cv2.Scharr(self.grid_map_grey, cv2.CV_32F, 0, 1,
                            borderType=cv2.BORDER_ISOLATED)

        free_pnts = np.asarray(np.where(frontier_mat == 255)).T.tolist()
        frontier_mat = np.zeros(np.shape(self.grid_map_grey), dtype=np.uint8)
        row, col = np.shape(self.grid_map_grey)

        for j in range(len(free_pnts)):
            r, c = free_pnts[j]
            if self.grid_map_grey[r, c] == 255:
                for i in range(8):
                    r1 = r + dx[i]
                    c1 = c + dy[i]

                    if 0 <= r1 < row and 0 <= c1 < col:
                        if self.grid_map_grey[r1, c1] == 128:
                            frontier_mat[r, c] = 255
                            break
            elif self.grid_map_grey[r, c] == 128:
                for i in range(8):
                    r1 = r + dx[i]
                    c1 = c + dy[i]

                    if 0 <= r1 < row and 0 <= c1 < col:
                        if self.grid_map_grey[r1, c1] == 255:
                            frontier_mat[r1, c1] = 255
                            # break

        # kernel = np.ones((5, 5), np.uint8) * 255
        # frontier_mat = cv2.dilate(frontier_mat, kernel, iterations=3)
        #
        # image_message = self.bridge.cv2_to_imgmsg(frontier_mat,
        #                                           encoding="mono8")
        # self.debug_pub.publish(image_message)

        numLabels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mat, 8)

        self.frontier_num_labels = numLabels
        self.frontier_centroid = centroids
        self.frontier_stat = stats

        s = int(math.ceil(self.agent_radius / self.resolution))
        kernel = np.zeros((2 * s + 1, 2 * s + 1), np.uint8)
        cv2.circle(kernel, (s, s), s, 1, -1)

        obstacle_grids = (self.grid_map_grey == 0).astype('uint8')
        obstacle_grids = cv2.dilate(obstacle_grids, kernel, iterations=1,
                                    borderType=cv2.BORDER_ISOLATED).astype(
            bool)
        self.grid_map_grey[obstacle_grids] = 0

        for i in range(1, numLabels):
            stat = stats[i]
            area = stat[cv2.CC_STAT_AREA]
            indices = (labels == i)
            if area <= 15:
                frontier_mat[indices] = 0
                labels[indices] = 0

        frontier_mat[obstacle_grids] = 0
        labels[obstacle_grids] = 0

        grad_x[frontier_mat == 0] = 0
        grad_y[frontier_mat == 0] = 0

        iteration = 0
        iter_max = 2
        kernel = np.asarray([[0, 1, 0], [1, 0, 1], [0, 1, 0]]).astype(np.uint8)
        frontier_new = np.zeros_like(frontier_mat)
        invalid_frontier = np.logical_or(obstacle_grids,
                                         self.grid_map_grey == 128)

        frontier_mat = labels.astype(float)
        while iteration < iter_max:
            frontier_new = cv2.dilate(frontier_mat, kernel, iterations=1,
                                      borderType=cv2.BORDER_ISOLATED)
            grad_x = cv2.filter2D(grad_x, ddepth=-1, kernel=kernel,
                                  borderType=cv2.BORDER_ISOLATED)
            grad_x = cv2.filter2D(grad_x, ddepth=-1, kernel=kernel,
                                  borderType=cv2.BORDER_ISOLATED)

            frontier_new[invalid_frontier] = 0
            grad_x[invalid_frontier] = 0
            grad_y[invalid_frontier] = 0

            iteration += 1
            if iteration == iter_max:
                frontier_new[np.logical_and(frontier_new > 0,
                                            frontier_mat > 0)] = 0
            frontier_mat = frontier_new

        # self.frontier_map = (frontier_mat == 255).astype(float)
        self.frontier_map = frontier_mat.astype(float)
        self.frontier_map[obstacle_grids] = 0
        for obj_id in self.sightings_map:
            self.sightings_map[obj_id][obstacle_grids] = 0
            # self.grid_map_color[self.sightings_map[obj_id] > 0, :] = [189, 114, 0]

        self.grid_map_color[self.frontier_map > 0, :] = [48, 172, 119]
        self.frontier_phase = -cv2.phase(grad_x, grad_y) + np.pi

    def semantic_map_callback(self, msg):
        if self.pos_var is None:
            rospy.loginfo('Robot pose covariance is not set.')
            return

        if self.pos is None:
            rospy.loginfo('Robot pose is not set.')
            return

        if self.grid_map_color is None:
            rospy.loginfo('Grid map is not constructed.')
            return

        grid_map = self.grid_map_color.copy()
        startAngle = 0
        endAngle = 360

        pos_grid = self.to_grid(self.pos)

        axesLength = (
        max(int(round(5 * np.sqrt(self.pos_var) / self.resolution)), 1),
        max(int(round(5 * np.sqrt(self.pos_var) / self.resolution)), 1))
        angle = 0
        # Red color in BGR
        color = (0, 0, 255)
        # Line thickness of 5 px
        thickness = 1
        cv2.ellipse(grid_map, pos_grid, axesLength,
                    angle, startAngle, endAngle, color, thickness)

        colors = [(142, 47, 126), (0, 255, 0)]
        # grid_map = np.ones_like(grid_map)*255
        for obj_msg in msg.objects:
            obj_id = obj_msg.id
            covariance = np.reshape(obj_msg.covariance, (2, 2))
            obj_pose = np.asarray([obj_msg.x, obj_msg.y])
            class_probs = np.asarray(obj_msg.probability)

            if obj_id not in self.objects:
                self.objects[obj_id] = MapObject(obj_id, obj_pose, covariance,
                                                 class_probs)
            else:
                obj = self.objects[obj_id]
                obj.update(obj_pose, covariance, class_probs)

            center_coordinates = self.to_grid(obj_pose)
            color = colors[0]

            w, v = eig(covariance)
            if w[0] < w[1]:
                axesLength = (
                    max(int(round(5 * np.sqrt(w[1]) / self.resolution)), 1),
                    max(int(round(5 * np.sqrt(w[0]) / self.resolution)), 1))

                angle = np.arctan2(v[1, 1], v[0, 1]) * 180.0 / np.pi
            else:
                axesLength = (
                    max(int(round(5 * np.sqrt(w[0]) / self.resolution)), 1),
                    max(int(round(5 * np.sqrt(w[1]) / self.resolution)), 1))

                angle = np.arctan2(v[1, 0], v[0, 0]) * 180.0 / np.pi

            cv2.ellipse(grid_map, center_coordinates, axesLength,
                        angle, startAngle, endAngle, color,
                        thickness)

            p2 = self.pos + 10 * self.resolution * np.asarray(
                [np.cos(self.ang),
                 np.sin(self.ang)])
            p2 = self.to_grid(p2)
            # .arrowedLine(grid_map, pos_grid, p2, color=(32, 177, 237))

        total_text_width = 0
        total_text_height = 0
        text_color_bg = (255, 255, 255)
        text_color = (0, 0, 0)

        font = cv2.FONT_HERSHEY_PLAIN
        font_scale = 1  # 0.6
        font_thickness = 1

        for obj_id, obj in self.objects.items():
            if obj.visualized:
                text1 = 'object ID: ' + str(obj_id)

                text2 = 'pose: ' + str("%.2f" % obj.pos[0]) + ', ' + \
                        str("%.2f" % obj.pos[1])

                text3 = 'pose var: ' + str(
                    "%.2f" % (obj.pos_var[0, 0] * 10000.0)) + ', ' + \
                        str("%.2f" % (
                            obj.pos_var[0, 1] * 10000.0)) + ', ' + \
                        str("%.2f" % (
                            obj.pos_var[1, 0] * 10000.0)) + ', ' + \
                        str("%.2f" % (obj.pos_var[1, 1] * 10000.0))

                class_max = np.argmax(obj.class_probs)
                text4 = self.classes[class_max] + ': ' + str(
                    "%.2f" % obj.class_probs[class_max])

                text5 = ''

                texts = [text1, text2, text3, text4, text5]

                for text in texts:
                    text_size, _ = cv2.getTextSize(text, font, font_scale,
                                                   font_thickness)
                    text_w, text_h = text_size
                    total_text_height += (text_h + 1)
                    if text_w > total_text_width:
                        total_text_width = text_w

                # text_coord = list(center_coordinates)
                # if text_coord[0] > self.width/2 and text_coord[1] > self.height/2:
                #     text_coord = [text_coord[0] - total_text_width,
                #                   text_coord[1] - total_text_height]
                # elif text_coord[0] > self.width/2 and text_coord[1] <= self.height/2:
                #     text_coord = [text_coord[0] - total_text_width,
                #                   text_coord[1]]
                # elif text_coord[0] <= self.width/2 and text_coord[1] <= self.height/2:
                #     text_coord = [text_coord[0], text_coord[1]]
                # else:
                #     text_coord = [text_coord[0], text_coord[1] - total_text_height]

        if total_text_width and total_text_height:
            obj_info_image = np.zeros((total_text_height, total_text_width),
                                      dtype=np.uint8)
            text_coord = [0, 0]
            for obj_id, obj in self.objects.items():
                # cv2.rectangle(grid_map, (text_coord[0], text_coord[1]),
                #               (text_coord[0] + total_text_width,
                #                text_coord[1] + total_text_height),
                #               text_color_bg, -1)
                if obj.visualized:
                    text1 = 'object ID: ' + str(obj_id)

                    text2 = 'pose: ' + str("%.2f" % obj.pos[0]) + ', ' + \
                            str("%.2f" % obj.pos[1])

                    text3 = 'pose var: ' + str(
                        "%.2f" % (obj.pos_var[0, 0] * 10000.0)) + ', ' + \
                            str("%.2f" % (
                                obj.pos_var[0, 1] * 10000.0)) + ', ' + \
                            str("%.2f" % (
                                obj.pos_var[1, 0] * 10000.0)) + ', ' + \
                            str("%.2f" % (obj.pos_var[1, 1] * 10000.0))

                    class_max = np.argmax(obj.class_probs)
                    text4 = self.classes[class_max] + ': ' + str(
                        "%.2f" % obj.class_probs[class_max])

                    text5 = ''

                    texts = [text1, text2, text3, text4, text5]

                    for text in texts:
                        text_size, _ = cv2.getTextSize(text, font, font_scale,
                                                       font_thickness)
                        _, text_h = text_size
                        text_coord[1] = text_coord[1] + text_h + 1
                        cv2.putText(obj_info_image, text, tuple(text_coord),
                                    font,
                                    font_scale, 255, font_thickness)

            image_message = self.bridge.cv2_to_imgmsg(obj_info_image,
                                                      encoding="mono8")
            self.object_info_pub.publish(image_message)

        # if self.goal is not None:
        # cv2.drawMarker(grid_map, self.to_grid(self.goal),
        # color=(25, 83, 217), markerSize=10)
        # print('pos grid is ', pos_grid)
        # print('grid map shape is ', grid_map.shape)
        if self.policy is not None:
            path = set()
            pos_grid_old = np.asarray(self.to_grid_old(self.pos))
            # print('pos is ', self.pos)
            # print('pos grid old is ', pos_grid_old)
            while True:
                grid_map[pos_grid[1], pos_grid[0], :] = [189, 114, 0]
                if tuple(pos_grid_old) in path:
                    break
                else:
                    path.add(tuple(pos_grid_old))
                pos_index = np.ravel_multi_index(np.flipud(pos_grid_old),
                                                 (self.old_height,
                                                  self.old_width))
                if self.policy.get(pos_index, -1) < 0:
                    print('this grid has no policy')
                    break
                else:
                    action_index = self.policy[pos_index]
                    pos_grid_old = pos_grid_old + np.flipud(
                        self.action_deltas[action_index])
                    pos_map = self.to_map_old(pos_grid_old)
                    pos_grid = self.to_grid(pos_map)

        # for obj_id in self.sightings_map:
        #     obj = self.objects[obj_id]
        #     obj_pose = obj.pos
        #
        #     center_coordinates = self.to_grid(obj_pose)
        #     color = colors[0]
        #
        #     w, v = eig(obj.pos_var)
        #     if w[0] < w[1]:
        #         axesLength = (
        #             max(int(round(5 * np.sqrt(w[1]) / self.resolution)), 1),
        #             max(int(round(5 * np.sqrt(w[0]) / self.resolution)), 1))
        #
        #         angle = np.arctan2(v[1, 1], v[0, 1])* 180.0 /np.pi
        #     else:
        #         axesLength = (
        #             max(int(round(5 * np.sqrt(w[0]) / self.resolution)), 1),
        #             max(int(round(5 * np.sqrt(w[1]) / self.resolution)), 1))
        #
        #         angle = np.arctan2(v[1, 0], v[0, 0])* 180.0 /np.pi
        #
        #     cv2.ellipse(grid_map, center_coordinates, axesLength,
        #                            angle, startAngle, endAngle, color,
        #                            thickness)

        image_message = self.bridge.cv2_to_imgmsg(grid_map,
                                                  encoding="bgr8")
        image_message.header.seq = self.seq
        image_message.header.stamp = rospy.get_rostime()
        image_message.header.frame_id = 'map'
        self.image_pub.publish(image_message)
        self.seq += 1


if __name__ == '__main__':
    rospy.init_node("planner")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Planner()
    try:
        whatever.run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
