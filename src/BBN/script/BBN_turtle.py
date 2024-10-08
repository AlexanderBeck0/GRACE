#!/usr/bin/env python3
from pybbn.graph.dag import Bbn
from pybbn.graph.edge import Edge, EdgeType
from pybbn.graph.jointree import EvidenceBuilder, EvidenceType
from pybbn.graph.node import BbnNode
from pybbn.graph.variable import Variable
from pybbn.pptc.inferencecontroller import InferenceController
from pybbn.graph.potential import Potential
from object_ros_msgs.srv import BBNInfer, BBNInferResponse
from time import time
from sensor_msgs.msg import CompressedImage
import rospy
import numpy as np
import os
# create the nodes


class BBN:
    def __init__(self):
        self.bathroom_classes = []
        self.bathroom_evidences = {}
        self.join_tree_bathroom = None
        self.create_bathroom_bbn()

        self.kitchen_classes = []
        self.kitchen_evidences = {}
        self.join_tree_kitchen = None
        self.create_kitchen_bbn()

        self.livingroom_classes = []
        self.livingroom_evidences = {}
        self.join_tree_livingroom = None
        self.create_livingroom_bbn()

        self.office_classes = []
        self.office_evidences = {}
        self.join_tree_office = None
        self.create_office_bbn()

        self.s = rospy.Service('BBN_infer', BBNInfer, self.handle_BBN_infer)
        self.semantic_label = [0.25, 0.25, 0.25, 0.25]
        self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage, self.img_callback)

    def img_callback(self, msg):
        # Get or create RoomLabel.txt
        file = open(os.path.join(os.path.dirname(__file__), "../../../out/RoomLabels/RoomLabel.txt"), "a")
        line = str(msg.header.seq) + " "
        for p in self.semantic_label:
            line += str(p)
            line += " " 

        file.write(line  + '\n')
        file.close()

        return

    def create_bathroom_bbn(self):
        a = BbnNode(Variable(0, 'bathroom', ['on', 'off']), [0.25, 0.75])
        b = BbnNode(Variable(1, 'toilet', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        c = BbnNode(Variable(2, 'hair drier', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        d = BbnNode(Variable(3, 'sink', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        e = BbnNode(Variable(4, 'toothbrush', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])

        # create the network structure
        bbn_bathroom = Bbn() \
            .add_node(a) \
            .add_node(b) \
            .add_node(c) \
            .add_node(d) \
            .add_node(e) \
            .add_edge(Edge(a, b, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, c, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, d, EdgeType.DIRECTED)) \
            .add_edge(Edge(d, e, EdgeType.DIRECTED))
        
        # convert the BBN to a join tree
        self.join_tree_bathroom = InferenceController.apply(bbn_bathroom)

        self.bathroom_classes = ['toilet', 'hair drier', 'sink', 'toothbrush']
        for evidence_class in self.bathroom_classes:
            self.bathroom_evidences[evidence_class] = EvidenceBuilder() \
                .with_node(self.join_tree_bathroom.get_bbn_node_by_name(evidence_class)) \
                .with_evidence('on', 1) \
                .build()

    def create_kitchen_bbn(self):
        a = BbnNode(Variable(0, 'kitchen', ['on', 'off']), [0.25, 0.75])
        b = BbnNode(Variable(1, 'oven', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        c = BbnNode(Variable(2, 'toaster', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        d = BbnNode(Variable(3, 'sink', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        e = BbnNode(Variable(4, 'refrigerator', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        f = BbnNode(Variable(5, 'microwave', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        g = BbnNode(Variable(6, 'bottle', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])

        # create the network structure
        bbn_kitchen = Bbn() \
            .add_node(a) \
            .add_node(b) \
            .add_node(c) \
            .add_node(d) \
            .add_node(e) \
            .add_node(f) \
            .add_edge(Edge(a, b, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, c, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, d, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, e, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, f, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, g, EdgeType.DIRECTED))  
        
        
        # convert the BBN to a join tree
        self.join_tree_kitchen = InferenceController.apply(bbn_kitchen)

        self.kitchen_classes = ['oven',  'toaster', 'sink', 'refrigerator', 'microwave']
        for evidence_class in self.kitchen_classes:
            self.kitchen_evidences[evidence_class] = EvidenceBuilder() \
                .with_node(self.join_tree_kitchen.get_bbn_node_by_name(evidence_class)) \
                .with_evidence('on', 1) \
                .build()
            
    def create_livingroom_bbn(self):
        a = BbnNode(Variable(0, 'livingroom', ['on', 'off']), [0.25, 0.75])
        b = BbnNode(Variable(1, 'chair', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        c = BbnNode(Variable(2, 'sofa', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        d = BbnNode(Variable(3, 'diningtable', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        e = BbnNode(Variable(4, 'tvmonitor', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        f = BbnNode(Variable(5, 'remote', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])

        # create the network structure
        bbn_livingroom = Bbn() \
            .add_node(a) \
            .add_node(b) \
            .add_node(c) \
            .add_node(d) \
            .add_node(e) \
            .add_node(f) \
            .add_edge(Edge(a, b, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, c, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, d, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, e, EdgeType.DIRECTED)) \
            .add_edge(Edge(e, f, EdgeType.DIRECTED))  
        
        # convert the BBN to a join tree
        self.join_tree_livingroom = InferenceController.apply(bbn_livingroom)

        self.livingroom_classes = ['chair', 'sofa', 'diningtable', 'tvmonitor', 'remote']
        for evidence_class in self.livingroom_classes:
            self.livingroom_evidences[evidence_class] = EvidenceBuilder() \
                .with_node(self.join_tree_livingroom.get_bbn_node_by_name(evidence_class)) \
                .with_evidence('on', 1) \
                .build()
    
    def create_office_bbn(self):
        a = BbnNode(Variable(0, 'office', ['on', 'off']), [0.25, 0.75])
        b = BbnNode(Variable(1, 'chair', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        c = BbnNode(Variable(2, 'tvmonitor', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        d = BbnNode(Variable(3, 'laptop', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        e = BbnNode(Variable(4, 'book', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        f = BbnNode(Variable(5, 'mouse', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])
        g = BbnNode(Variable(6, 'keyboard', ['on', 'off']), [0.8, 0.2, 0.2, 0.8])

        # create the network structure
        bbn_office = Bbn() \
            .add_node(a) \
            .add_node(b) \
            .add_node(c) \
            .add_node(d) \
            .add_node(e) \
            .add_node(f) \
            .add_node(g) \
            .add_edge(Edge(a, b, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, c, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, d, EdgeType.DIRECTED)) \
            .add_edge(Edge(a, e, EdgeType.DIRECTED)) \
            .add_edge(Edge(d, f, EdgeType.DIRECTED)) \
            .add_edge(Edge(d, g, EdgeType.DIRECTED))
        
        # convert the BBN to a join tree
        self.join_tree_office = InferenceController.apply(bbn_office)

        self.office_classes = ['office', 'chair', 'tvmonitor', 'laptop', 'book', 'mouse', 'keyboard']
        for evidence_class in self.office_classes:
            self.office_evidences[evidence_class] = EvidenceBuilder() \
                .with_node(self.join_tree_office.get_bbn_node_by_name(evidence_class)) \
                .with_evidence('on', 1) \
                .build()

    def handle_BBN_infer(self, req):
        for evidence_class in req.evidences:
            if evidence_class in self.bathroom_classes:
                self.join_tree_bathroom.set_observation(self.bathroom_evidences[evidence_class])

            if evidence_class in self.kitchen_classes:
                self.join_tree_kitchen.set_observation(self.kitchen_evidences[evidence_class])

            if evidence_class in self.livingroom_classes:
                self.join_tree_livingroom.set_observation(self.livingroom_evidences[evidence_class])
            
            if evidence_class in self.office_classes:
                self.join_tree_office.set_observation(self.office_evidences[evidence_class])


        p1 = self.join_tree_bathroom.get_bbn_potential(
            self.join_tree_bathroom.get_bbn_node_by_name('bathroom'))
        p1 = Potential.to_dict([p1])
        p1 = p1['0=on']

        p2 = self.join_tree_kitchen.get_bbn_potential(
            self.join_tree_kitchen.get_bbn_node_by_name('kitchen'))
        p2 = Potential.to_dict([p2])
        p2 = p2['0=on']

        p3 = self.join_tree_livingroom.get_bbn_potential(
            self.join_tree_livingroom.get_bbn_node_by_name('livingroom'))
        p3 = Potential.to_dict([p3])
        p3 = p3['0=on']

        p4 = self.join_tree_office.get_bbn_potential(
            self.join_tree_office.get_bbn_node_by_name('office'))
        p4 = Potential.to_dict([p4])
        p4 = p4['0=on']
        
        self.join_tree_bathroom.unobserve_all()
        self.join_tree_kitchen.unobserve_all()
        self.join_tree_livingroom.unobserve_all()
        self.join_tree_office.unobserve_all()

        self.semantic_label = [p1, p2, p3, p4]

        p5 = self.join_tree_kitchen.get_bbn_potential(
        self.join_tree_kitchen.get_bbn_node_by_name('microwave'))
        p5 = Potential.to_dict([p5])
        p5 = p5['5=on']
        
        return BBNInferResponse(p5)


if __name__ == '__main__':
    rospy.init_node("BBN")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = BBN()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
