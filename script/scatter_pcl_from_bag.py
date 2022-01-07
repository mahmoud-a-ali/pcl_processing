import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


import rospy
import pcl
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

from sensor_msgs import point_cloud2



import rosbag
bag = rosbag.Bag('/home/robotics/Mahmoud_ws/github/sgp/rosbags/distance_metric/cave_sideway_1.bag')



def convert_spherical_2_cartesian(theta, alpha, dist):
    x = np.array( dist * np.sin(alpha) * np.cos(theta), dtype='float32').reshape(-1,1)
    y = np.array( dist * np.sin(alpha) * np.sin(theta), dtype='float32').reshape(-1,1)
    z = np.array( dist * np.cos(alpha) , dtype='float32').reshape(-1,1)
    return x, y, z


def convert_cartesian_2_spherical(x, y, z):
    dist = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arctan2(y, x)
    alpha = np.arccos(z / dist)
    return theta, alpha, dist




Xs = []
Ys = []
Zs = []
Ocs = []

# for topic, msg, t in bag.read_messages(topics=['org_occ_srfc', 'sgp_occ_srfc']):
for topic, pc_msg, t in bag.read_messages(topics=['/org_low_frq_pcl', '/occ_surface']):
	pcl_arr = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg, squeeze = True)       
	pcl_arr = np.array(pcl_arr.tolist()) 
	pcl_arr = pcl_arr.transpose()
	print("shape: ", np.shape(pcl_arr) )

	########## extract XYZI ##########
	x = pcl_arr[:][0].reshape(-1,1) 
	y = pcl_arr[:][1].reshape(-1,1) 
	z = pcl_arr[:][2].reshape(-1,1) 
	oc = pcl_arr[:][3].reshape(-1,1) 

	########## limit to range ##########
	ids = np.where(oc < 5)[0]
	print("occ > x: ids ", np.shape(ids) )
	x = x[ids] 
	y = y[ids] 
	z = z[ids] 
	oc = oc[ids][:] 

	########## save to compare ##########
	Xs.append( x )
	Ys.append( y )
	Zs.append( z )
	Ocs.append( oc )


bag.close()



ax = plt.axes(projection='3d')
for ind in range(0, 2):
	thetas, alphas, occs = convert_cartesian_2_spherical(Xs[ind], Ys[ind], Ocs[ind])
	ax.scatter(thetas, alphas, occs, s= 1.5)#, c='r') #, c = rgb/255, s=0.01)


plt.show()


