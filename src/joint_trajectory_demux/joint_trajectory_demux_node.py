#!/usr/bin/env python
import rospy
import joint_trajectory_demux

if __name__ == "__main__":
    rospy.init_node("joint_trajectory_demux")

    demux = joint_trajectory_demux.JointTrajectoryDemux("/", "~")
    # Wait for controllers to start up
    rospy.sleep(5)
    # demux.load_controllers(self.pns + "controller_list")
    demux.discover_controllers()
    rospy.spin()
