#!/usr/bin/env python

import rospy
import intera_interface
import intera_control
import sys
import argparse
import numpy as np

from sensor_msgs.msg import JointState

import testing_func

class Limb_Control:
    def __init__(self, limb_name):
        """
        A class container of robot's limb information, PID controller
        and velocity control.
        """
        self.error = 0
        self.robot_limb = intera_interface.Limb(limb_name)
        self.joints_pid = {name: intera_control.PID() for name in self.robot_limb.joint_names()}
        # FIXIT:
        # put in valid topic name, valid mas type,
        self.error_sub = rospy.Subscriber('', XXX, self.get_error)

    def get_error(self, data):
        self.error = data

    def set_joint_pid_k(self, joint_name, kp, ki, kd):
        self.joints_pid[joint_name].set_kp(kp)
        self.joints_pid[joint_name].set_ki(ki)
        self.joints_pid[joint_name].set_kd(kd)

    def control_joint(self, joint_name, error):
        """
        set joint velocity
        """
        vel = joints_pid[joint_name].compute_output(error)
        joint_command = {joint_name: vel}
        self.robot_limb.set_joint_velocities(joint_command)

    def start_control(self):
        #subscribe to error
        while not rospy.is_shutdown():

            if error > 0.01:
                self.control_joint('right_j5', self.error) #control joint 5 vel
                self.control_joint('right_j4', self.error) #control joint 6 vel

            rospy.Rate(30).sleep()

    def init_all_joints(self):
        """
        Initialize all joints to starting post
        """
        joints_starting_angles = [0.0, 1.1, 0.0, -1.1, 0.0, -np.pi/2, 0]
        joint_command = dict()

        count = 0
        for name in self.robot_limb.joint_names():
            joint_command.update({name: joints_starting_angles[count]})
            count+=1

        done = False
        pos_goal = 0b1111111

        while not done and not rospy.is_shutdown():
            self.robot_limb.set_joint_positions(joint_command)

            pos = 0b0000001
            pos_curr = 0b0000000
            for name in self.robot_limb.joint_names():
                if np.absolute(joint_command[name] - self.robot_limb.joint_angle(name)) < 0.005:
                    pos_curr = pos_curr | pos

                if pos_curr == pos_goal:
                    done = True

                pos = pos << 1

            rospy.Rate(1).sleep()

    def set_joints_zeros(self):
        """
        Set all joints to 0 position
        """

        done = False
        pos_goal = 0b1111111

        while not done and not rospy.is_shutdown():
            joint_command = {name: 0.0 for name in self.robot_limb.joint_names()}
            self.robot_limb.set_joint_positions(joint_command)

            pos = 0b0000001
            pos_curr = 0b0000000
            for name in self.robot_limb.joint_names():
                if np.absolute(0.0 - self.robot_limb.joint_angle(name)) < 0.005:
                    pos_curr = pos_curr | pos

                if  pos_curr == pos_goal:
                    done = True

                pos = pos << 1

            rospy.Rate(1).sleep()

if __name__ == "__main__":

    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. Exiting."), "ERROR")
        sys.exit()

    print("Initializing node... ")
    rospy.init_node("laby_mani")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    init_state = rs.state().enabled

    rospy.loginfo("Enabling robot...")
    rs.enable()

    # do stuff
    Sawyerlimb = Limb_Control(valid_limbs[0])
    Sawyerlimb.set_joints_zeros()
    Sawyerlimb.init_all_joints()
