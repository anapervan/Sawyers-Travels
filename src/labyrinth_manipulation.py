import rospy
import intera_interface
import intera_control
import sys

class Limp_Control:
    def __init__(self, limp_name):
        """
        """

        self.robot_limp = intera_interface.Limp(limp_name)
        self.joint_names = self.robot_limp.joint_names()
        self.pid = intera_control.PID()
        # self.joint_angles = self.robot_limp.joint_angles()
        # self.joint_vels = self.robot_limp.joint_vels()

    def set_joint_vel(self, joint_name, vel):
        joint_command = {joint_name: vel}
        self.robot_limb.set_joint_positions(joint_command)

    def control(self, joint_name, error):
        vel = self.pid.compute_ouput(error)
        self.set_joint_vel(joint_name, vel)


if __name__ == "main":

    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        sys.exit()

    print("Initializing node... ")
    rospy.init_node("laby_mani")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()

    # do stuff
    SawyerLimp = Limp_Control(valid_limbs)
