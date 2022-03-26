#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Actuator speed (deg/s)
SPEED = 10


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check


def example_move_to_start_position(base):
    print("enter the function")
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    constrained_joint_angles = Base_pb2.ConstrainedJointAngles()
    print(type(constrained_joint_angles),'this is the type of joint angles')
    print('this is the constraints angles',constrained_joint_angles.joint_angles.joint_angles)
    print('this is the constraints angles',constrained_joint_angles.constraint)


    actuator_count = base.GetActuatorCount().count
    print('this is actuator count', actuator_count)
    angles = [10] * actuator_count
    
    # Actuator 4 at 90 degrees
    for joint_id in range(len(angles)):
        joint_angle = constrained_joint_angles.joint_angles.joint_angles.add()
        print('this is the type ',type(joint_angle))
        print('this is the joint angle',joint_angle.value)
        # the index of joints
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angles[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Reaching joint angles...")
    base.PlayJointTrajectory(constrained_joint_angles)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Joint angles reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_get_joint_angle(base):
    print("Getting Angles for every joint...")
    print("Joint ID : Joint Angle")
    input_joint_angles = base.GetMeasuredJointAngles()
    for joint_angle in input_joint_angles.joint_angles:
        print(joint_angle.joint_identifier, " : ", joint_angle.value)
    joint_vel=base.GetActuatorCount()
    print(joint_vel)
    print(base.GetAllJointsSpeedHardLimitation())
    print(base.GetMeasuredCartesianPose())
    print(base.GetArmState())
    # print("Computing Foward Kinematics using joint angles...")
    # print(pose)


def example_send_joint_speeds(base):
    joint_speeds = Base_pb2.JointSpeeds()
    print('this is joint speed',joint_speeds)
    actuator_count = base.GetActuatorCount().count
    # The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
        ss = [SPEED, 0, -SPEED, 0, SPEED, 0, -SPEED]
        speedss=[ss]*1000
        before=time.time()
        for j in range(len(speedss)):
            speeds=speedss[j]
            i = 0
            joint_speeds = Base_pb2.JointSpeeds()
            for speed in speeds:
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = i
                joint_speed.value = speed
                joint_speed.duration = 0
                i = i + 1
            # print("Sending the joint speeds for 10 seconds...")
            base.SendJointSpeedsCommand(joint_speeds)
        end=time.time()
        print('run 1000 commend spend ',end-before)
        # time.sleep(3)
    print("Stopping the robot")
    base.Stop()

    return True


def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)

        # Example core
        success = True
        # success &= example_move_to_start_position(base)
        # success &= example_send_joint_speeds(base)
        example_get_joint_angle(base)
        return 0 if success else 1


if __name__ == "__main__":
    exit(main())
