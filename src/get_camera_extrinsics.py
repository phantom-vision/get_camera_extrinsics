#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import pdb

def get_image_points(K,pts_in_alvar_frame):
    epsilon = 0.0000001
    for i in range(pts_in_alvar_frame.shape[1]):
        if(i==0):
            image_points = np.reshape(np.matmul(K,pts_in_alvar_frame[:,i]),(3,1))
        else:
            image_points = np.hstack((image_points,np.reshape(np.matmul(K,pts_in_alvar_frame[:,i]),(3,1))))
    image_points = image_points/ (image_points[-1,:] + epsilon)
    return image_points[:-1,:]

def get_extrinsics(K,pts_in_alvar_frame,pts_in_world_frame):
    rospy.loginfo("In get_extrinsics")
    image_points = get_image_points(K,pts_in_alvar_frame)


    return H

if __name__ == '__main__':
    rospy.init_node('get_camera_extrinsics', anonymous = True)
    K = np.load("real_sense_intrinsics.npy")
    pts_in_alvar_frame = np.load("pts_in_alvar_frame.npy")
    pts_in_world_frame = np.load("pts_in_world_frame.npy")
    H = get_extrinsics(K,pts_in_alvar_frame,pts_in_world_frame)