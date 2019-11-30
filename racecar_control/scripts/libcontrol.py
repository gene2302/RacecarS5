#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms((trans1, rot1), (trans2, rot2)):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    
    return (trans3, rot3)
    
    # do brushfire algorithm here
def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
    
    mapOfWorld[occupancyGrid==100] = 1 # obstacles
    mapOfWorld[occupancyGrid==-1] = 1  # unknowns
    (size_x, size_y) = (mapOfWorld.shape)
    
    # do brushfire algorithm here
        # Get boundaries
    min_x = 0
    max_x = size_x
    min_y = 0
    max_y = size_y
    finished = False
    checked_value = 0

    while not finished:
        finished = True
        checked_value += 1
	#print(checked_value)
	#print("max value in grid " + str(np.amax(mapOfWorld)))
        for x_counter in range(min_x, max_x):
	    #print("yo")
            for y_counter in range(min_y, max_y):
		#Valeur a analyser trouvee
                if mapOfWorld[x_counter][y_counter] == checked_value:
			#print(checked_value)
			#analyse des valeurs adjacentes en X positif
			if x_counter < (max_x-1):
				if mapOfWorld[x_counter+1][y_counter] == 0 :
					mapOfWorld[x_counter+1][y_counter] = checked_value+1
					finished = False
			#analyse des valeurs adjacentes en X negatif
			if x_counter > 0:
				if mapOfWorld[x_counter-1][y_counter] == 0 :
					mapOfWorld[x_counter-1][y_counter] = checked_value+1
					finished = False
			#analyse des valeurs adjacentes en Y positif
			if y_counter < (max_y-1):
				if mapOfWorld[x_counter][y_counter+1] == 0 :
					mapOfWorld[x_counter][y_counter+1] = checked_value+1
					finished = False
			#analyse des valeurs adjacentes en Y negatif
			if y_counter > 0:
				if mapOfWorld[x_counter][y_counter-1] == 0:
					mapOfWorld[x_counter][y_counter-1] = checked_value+1    
					finished = False  
                    
    
    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld    

