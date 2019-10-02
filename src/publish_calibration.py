#!/usr/bin/env python

import rospy, rospkg, tf
import os, sys, yaml
import numpy as np
import scipy

def read_calibration():
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('extrinsic_calibration')
    filename = package_path + "/calib_results/calibration_result.yaml"

    with open(filename, 'r') as stream:
        calib_data = yaml.safe_load(stream)

    return calib_data

def rot_to_quat(rmat):
    qw = 0.5 * np.sqrt(1.0 + rmat[0,0] + rmat[1,1] + rmat[2,2])
    qx = (rmat[2,1] - rmat[1,2]) / (4.0 * qw)
    qy = (rmat[0,2] - rmat[2,0]) / (4.0 * qw)
    qz = (rmat[1,0] - rmat[0,1]) / (4.0 * qw)
    return np.array([qx, qy, qz, qw])

def clean(lst):
    lastelem = ''.join(list(lst[-1])[0:-1])
    newlst = lst[1:len(lst)]
    newlst[-1] = lastelem
    return newlst

if __name__ == '__main__':
    rospy.init_node('extrinsic_calib_tf_broadcaster')

    # Read parameters from yaml file
    calib_dict = read_calibration()
    Rmat = clean(calib_dict['Rmat'].split())
    Tvec = clean(calib_dict['Tvec'].split())

    Rmat = np.reshape(np.array(map(float, Rmat)), (3,3))
    Tvec = np.array(map(float, Tvec))

    Qvec = rot_to_quat(Rmat)

    board_frame = "board_frame"
    camera_frame = "camera_color_optical_frame"

    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        try: 
            br.sendTransform(1e-3 * Tvec, Qvec,
                         rospy.Time.now(),
                         board_frame, camera_frame)
        except (tf.ConnectivityException):
            continue

        rospy.Rate(10.0).sleep()


