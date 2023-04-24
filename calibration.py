#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 16:15:21 2020

@author: hp
"""

import rospy
import roslib
import tf
import numpy as np
import time

import urx
import math3d as m3d

def listen_tf(src, target):
    rospy.init_node('tf_listener', anonymous=True)
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform(target, src, rospy.Time(), rospy.Duration(5))
    t, q = tf_listener.lookupTransform(target, src, rospy.Time())
    
    o_m3d = m3d.Orientation()
    q_m3d = m3d.UnitQuaternion(q[3], q[0], q[1], q[2])
    o_m3d.set_quaternion(q_m3d)

    t_m3d = m3d.Vector(t[0], t[1], t[2])
    T_m3d = m3d.Transform(o_m3d, t_m3d)
    
    return T_m3d

rob = urx.Robot("158.132.172.214")

rob.set_tcp((0,-0.048, 0.043, 1.57, 0, 0))

marker_to_base = rob.get_pose()
print("marker_to_base")
print(marker_to_base)


camera_to_marker = listen_tf("camera_color_optical_frame", "ar_marker_avg")

print('camera_to_marker')
print('quaternion')
print(camera_to_marker)

#base_to_camera = base_to_marker * camera_to_marker.inverse()
camera_to_base = marker_to_base * camera_to_marker
print("camera_to_base")
print(camera_to_base)
rob.close()

transformation_array=camera_to_base.get_array()

camera_to_base_translation=[]
camera_to_base_translation.append(transformation_array[0][3])
camera_to_base_translation.append(transformation_array[1][3])
camera_to_base_translation.append(transformation_array[2][3])

orientation_matrix_array=np.vstack((np.vstack((transformation_array[0][0:3],
                                    transformation_array[1][0:3])),
                                    transformation_array[2][0:3]))

camera_to_base_orientation = m3d.Orientation()
camera_to_base_orientation.set_array(orientation_matrix_array)
camera_to_base_orientation_quaternion=camera_to_base_orientation.get_unit_quaternion()
print('quaternion')
print(camera_to_base_orientation_quaternion)
print('position')
print(camera_to_base_translation)




