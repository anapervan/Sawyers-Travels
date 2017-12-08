#!/usr/bin/env python

import rospy
import intera_interface
import intera_control
import sys
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
# import testing_func


class Limb_Control:

    def __init__(self, limb_name):
        """
        A class container of robot's limb information, PID controller
        and velocity control.
        """
        self.error = []
        self.joint_pos = {}
        self.robot_limb = intera_interface.Limb(limb_name)

        self.joints_pid = {name: intera_control.PID() for name in self.robot_limb.joint_names()}
        self.joints_vel_pid = {name: intera_control.PID() for name in self.robot_limb.joint_names()}

        self.joints_starting_angles = {"right_j0":0.0, "right_j1":1.1, "right_j2":0.0,
                            "right_j3":-1.1, "right_j4":0.010, "right_j5":-1.563, "right_j6":0.170}

        self.error_sub = rospy.Subscriber('ball_error', Float64MultiArray, self.get_error)
        self.joint_pos_sub = rospy.Subscriber('/robot/joint_states', JointState, self.get_joint_states,queue_size=1)

    def get_error(self, data):
        self.error = data.data

    def get_joint_states(self, data):
        self.joint_pos = {}
        for i in range(len(data.name)):
            self.joint_pos.update({data.name[i]: data.position[i]})

    def set_joint_pid_k(self, joint_name, kp, ki, kd):
        self.joints_pid[joint_name].set_kp(kp)
        self.joints_pid[joint_name].set_ki(ki)
        self.joints_pid[joint_name].set_kd(kd)

    def set_joint_vel_pid_k(self, joint_name, kp, ki, kd):
        self.joints_vel_pid[joint_name].set_kp(kp)
        self.joints_vel_pid[joint_name].set_ki(ki)
        self.joints_vel_pid[joint_name].set_kd(kd)

    def control_joint2(self, joint_errors):
        joint_command = {}

        for joint, error in joint_errors.iteritems():

            vel = 0.05 - np.absolute(self.joints_vel_pid[joint].compute_output(error))
            # print joint, " ", vel, " ", error
            if vel < 0.0:
                vel = 0.005
            SawyerLimb.robot_limb.set_joint_position_speed(vel)

            pos = self.joints_pid[joint].compute_output(error)

            if np.absolute(pos) <= 0.2:
                pos = self.joints_starting_angles[joint] + pos

            elif pos > 0.2:
                pos = self.joints_starting_angles[joint] + 0.2

            elif pos < -0.2:
                pos = self.joints_starting_angles[joint] - 0.2

            joint_command.update({joint: pos})

        self.robot_limb.set_joint_positions(joint_command)

    def control_joint(self, joint_errors):
        """
        set joint velocity
        """
        joint_command = {}

        for joint, error in joint_errors.iteritems():

            # print joint, " ", self.joint_pos[joint]-self.joints_starting_angles[joint]
            # print "  ", self.joint_pos[joint], " ", self.joints_starting_angles[joint]

            # if np.absolute(self.joint_pos[joint] - self.joints_starting_angles[joint]) < 0.07:
                vel = self.joints_pid[joint].compute_output(error)
                joint_command.update({joint: vel})
            # else:
            #     vel = 0.0
            #     joint_command.update({joint: vel})

        self.robot_limb.set_joint_velocities(joint_command)

    def start_control(self):
        # subscribe to error

        while not rospy.is_shutdown():
            joint_errors = {}

            if len(self.error) != 0:

                # if np.absolute(self.error[0]) > 0.002:
                #     if np.absolute(self.error[0]) < 0.07:
                #         joint_errors.update({'right_j5': -1*self.error[0]})  # control joint 5 vel
                #     elif self.error[0] >= 0.07:
                #         joint_errors.update({'right_j5': -0.07})
                #     elif self.error[0] <= -0.07:
                #         joint_errors.update({'right_j5': 0.07})
                #
                # else:
                #     joint_errors.update({'right_j5': 0.0})  # control joint 5 vel
                #
                # if np.absolute(self.error[1]) > 0.002:
                #     if np.absolute(self.error[1]) < 0.07:
                #         joint_errors.update({'right_j4': self.error[1]})  # control joint 6 vel
                #     elif self.error[1] >= 0.07:
                #         joint_errors.update({'right_j4': 0.07})
                #     elif self.error[1] <= -0.07:
                #         joint_errors.update({'right_j4': -0.07})
                # else:
                #     joint_errors.update({'right_j4': 0.0})  # control joint 6 vel

                joint_errors.update({'right_j5': -1*self.error[0]})
                joint_errors.update({'right_j4': self.error[1]})

                # if np.absolute(self.error[0]) > 1.0:
                #     joint_errors.update({'right_j5': -1*self.error[0]})
                # else:
                #     joint_errors.update({'right_j5': 0.0})  # control joint 5 vel
                #
                # if np.absolute(self.error[1]) > 1.0:
                #     joint_errors.update({'right_j4': self.error[1]})
                # else:
                #     joint_errors.update({'right_j4': 0.0})  # control joint 6 vel

                self.control_joint2(joint_errors)

            rospy.Rate(60).sleep()

    def init_all_joints(self):
        """
        Initialize all joints to starting post
        """
        joint_command = dict()

        count = 0
        for name in self.robot_limb.joint_names():
            joint_command = self.joints_starting_angles
            count += 1

        self.robot_limb.move_to_joint_positions(joint_command, 30.0)

        # done = False
        # pos_goal = 0b1111111
        # self.robot_limb.set_joint_position_speed(0.05)
        # while not done and not rospy.is_shutdown():
        #     self.robot_limb.set_joint_positions(joint_command)
        #
        #     pos = 0b0000001
        #     pos_curr = 0b0000000
        #     for name in self.robot_limb.joint_names():
        #         if np.absolute(joint_command[name] - self.robot_limb.joint_angle(name)) < 0.01:
        #             pos_curr = pos_curr | pos
        #
        #         if pos_curr == pos_goal:
        #             done = True
        #
        #         pos = pos << 1
        #
        #     rospy.Rate(5).sleep()

    def set_joints_zeros(self):
        """
        Set all joints to 0 position
        """
        joint_command = {name: 0.0 for name in self.robot_limb.joint_names()}
        self.robot_limb.move_to_joint_positions(joint_command, 30.0)

        # done = False
        # pos_goal = 0b1111111
        #
        # while not done and not rospy.is_shutdown():
        #     self.robot_limb.set_joint_positions(joint_command)
        #
        #     pos = 0b0000001
        #     pos_curr = 0b0000000
        #     for name in self.robot_limb.joint_names():
        #         if np.absolute(0.0 - self.robot_limb.joint_angle(name)) < 0.01:
        #             pos_curr = pos_curr | pos
        #
        #         if  pos_curr == pos_goal:
        #             done = True
        #
        #         pos = pos << 1
        #
        #     rospy.Rate(5).sleep()


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
    # init_state = rs.state().enabled

    rospy.loginfo("Enabling robot...")
    rs.enable()

    # do stuff
    SawyerLimb = Limb_Control(valid_limbs[0])
    # SawyerLimb.set_joints_zeros()
    # rospy.Rate(0.5).sleep()
    SawyerLimb.robot_limb.set_joint_position_speed(0.05)
    SawyerLimb.init_all_joints()

    SawyerLimb.set_joint_pid_k('right_j5', 0.01, 0.005, 0.0)
    SawyerLimb.set_joint_pid_k('right_j4', 0.01, 0.005, 0.0)

    SawyerLimb.set_joint_vel_pid_k('right_j5', 0.001, 0.0, 0.0)
    SawyerLimb.set_joint_vel_pid_k('right_j4', 0.001, 0.0, 0.0)

    # SawyerLimb.robot_limb.set_joint_position_speed(0.01)
    rospy.Rate(0.5).sleep()
    SawyerLimb.start_control()
