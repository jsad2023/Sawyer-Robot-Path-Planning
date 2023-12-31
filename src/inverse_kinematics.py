#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at # #     http://www.apache.org/licenses/LICENSE-2.0
# # Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This is python script is modified from a code example provided by Rethink Robotics.
# Date: 2023-12-10
# Name: Justin Sadler

"""
Intera RSDK Inverse Kinematics Example
"""
import rospy
import intera_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


def ik_service_client(x, y, z):
    """ 
    Perforsm inverse kinematics on a cartesian point in 3D space
    """
    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    final_pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x,
                    y=y,
                    z=z,
                ),
                orientation=Quaternion(
                    x=0.704020578925,
                    y=0.710172716916,
                    z=0.00244101361829,
                    w=0.00194372088834,
                ),
            ),
    )
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(final_pose)
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')
    rospy.loginfo("Running Simple IK Service")
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

    # Result is invalid
    if resp.result_type[0] <= 0:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return None

    limb_joints = dict(list(zip(resp.joints[0].name, resp.joints[0].position)))
    rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
    rospy.loginfo("------------------")
    rospy.loginfo("Response Message:\n%s", resp)
    return limb_joints


if __name__ == '__main__':
    rospy.init_node("rsdk_ik_service_client")
    joints = ik_service_client(0.450628752997, 0.161615832271, 0.217447307078)
    if joints:
        rospy.loginfo("Simple IK call passed!")
        limb = intera_interface.Limb('right') # get the right limb's current joint angles
        limb.move_to_joint_positions(joints) # Sawyer wants t stretch


    else:
        rospy.logerr("Simple IK call FAILED")
