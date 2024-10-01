import pickle
import matplotlib.pyplot as plt
with open('../../../out/error_log.pkl') as f:  
    vel_err, ang_err = pickle.load(f)

plt.figure(1)
plt.subplot(211)
plt.plot(list(range(len(vel_err))), vel_err)
plt.subplot(212)
plt.plot(list(range(len(ang_err))), ang_err)
plt.show()