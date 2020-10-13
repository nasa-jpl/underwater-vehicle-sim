import h5py
import numpy as np
import matplotlib.pyplot as plt

f = h5py.File('/Users/branch/projs/ocean_worlds/LRAUV_data/201306042013_201306050146.mat', mode='r')
prop_time = np.array(f['platform_x_velocity_wrt_ground']['time'])
prop_val = np.array(f['platform_x_velocity_wrt_ground']['value'])

dvl_time = np.array(f['platform_speed_wrt_propeller']['time'])
dvl_val = np.array(f['platform_speed_wrt_propeller']['value'])

ori_time = np.array(f['platform_orientation']['time'])
ori_val = np.array(f['platform_orientation']['value'])

plt.scatter(prop_time, prop_val, c='b')
plt.scatter(dvl_time, dvl_val, c='r')
plt.scatter(ori_time, ori_val, c='g')

plt.show()