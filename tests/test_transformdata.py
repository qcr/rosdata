#!/usr/bin/env python3

###############
### MODULES ###
###############

import pytest
import numpy as np

import spatialmath as sm

import rospy
from geometry_msgs.msg import Transform as ROSTransformMsg

from rosdata.core.rosbag_transformer import TransformData


#############
### TESTS ###
#############

def test_constructor_simple():

    # Create rospy time
    t = rospy.Time.from_sec(10)

    # Create Transform
    trans = ROSTransformMsg()
    trans.translation.x = 0
    trans.translation.y = 0
    trans.translation.z = 0
    trans.rotation.x = 0
    trans.rotation.y = 0
    trans.rotation.z = 0
    trans.rotation.w = 1

    # Create instance
    data = TransformData(t, trans)

    # Check properties
    assert data.timestamp == pytest.approx(10, 0.001)
    assert data.transform == sm.SE3()


def test_constructor_complex():

    # Create rospy time
    t = rospy.Time.from_sec(20.4321)

    # Create Transform
    trans = ROSTransformMsg()
    trans.translation.x = 1
    trans.translation.y = 2
    trans.translation.z = 3
    trans.rotation.x = -0.169
    trans.rotation.y = 0.561
    trans.rotation.z = -0.774
    trans.rotation.w = 0.240

    # Create instance
    data = TransformData(t, trans)

    # Check properties
    trans = sm.SE3([1, 2, 3])
    trans.A[:3, :3] = sm.base.q2r([0.240, -0.169, 0.561, -0.774])

    assert data.timestamp == pytest.approx(20.4321, 0.001)
    assert data.transform == trans


def test_constructor_incorrect_time_type():

    # Create rospy time
    t = 20.4321

    # Create Transform
    trans = ROSTransformMsg()
    trans.translation.x = 0
    trans.translation.y = 0
    trans.translation.z = 0
    trans.rotation.x = 0
    trans.rotation.y = 0
    trans.rotation.z = 0
    trans.rotation.w = 1

    # Create instance
    with pytest.raises(AttributeError):
        data = TransformData(t, trans)


def test_constructor_incorrect_transform_type():

    # Create rospy time
    t = rospy.Time.from_sec(20.4321)

    # Create Transform
    trans = sm.SE3([1, 2, 3])
    trans.A[:3, :3] = sm.base.q2r([0.240, -0.169, 0.561, -0.774])

    # Create instance
    with pytest.raises(AttributeError):
        data = TransformData(t, trans)