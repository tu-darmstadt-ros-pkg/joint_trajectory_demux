import rospy
import trajectory_msgs.msg


class Controller:
    def __init__(self):
        self.name = ""
        self.topic = ""
        self.joints = []
        self.publisher = None
        self.next_trajectory_msg = trajectory_msgs.msg.JointTrajectory()


class JointTrajectoryDemux:
    def __init__(self, ns, pns):
        self.ns = ns
        self.pns = pns
        self.controllers = []
        self.load_controllers(self.pns + "controller_list")

        self.trajectory_sub = rospy.Subscriber("~command", trajectory_msgs.msg.JointTrajectory, self.trajectory_cb)

    def load_controllers(self, ns):
        controllers = rospy.get_param(ns)
        if controllers is None:
            rospy.logerr("No controller definition found in ns '" + ns + "'")
            return

        for controller in controllers:
            c = Controller()
            c.name = controller["name"]
            c.topic = c.name + "/command"
            c.publisher = rospy.Publisher(c.topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)
            c.joints = controller["joints"]
            rospy.logdebug("Added controller '" + c.name + ".")
            rospy.logdebug("--- topic: " + c.topic)
            rospy.logdebug("--- joints: " + ",".join(c.joints))
            self.controllers.append(c)

    def trajectory_cb(self, trajectory_msg):
        # Fill next trajectory message of each controller with same trajectory points
        for point in trajectory_msg.points:
            for c in self.controllers:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.time_from_start = point.time_from_start
                c.next_trajectory_msg.points.append(p)

        # Insert positions
        for joint_idx, joint in enumerate(trajectory_msg.joint_names):
            controller = None
            for c in self.controllers:
                if joint in c.joints:
                    controller = c
                    break
            if controller is None:
                rospy.logerr("No joint trajectory controller found for joint '" + joint + "'")
                return

            controller.next_trajectory_msg.joint_names.append(joint)
            for point_idx, point in enumerate(trajectory_msg.points):
                if joint_idx < len(point.positions):
                    controller.next_trajectory_msg.points[point_idx].positions.append(point.positions[joint_idx])
                if joint_idx < len(point.velocities):
                    controller.next_trajectory_msg.points[point_idx].velocities.append(point.velocities[joint_idx])
                if joint_idx < len(point.accelerations):
                    controller.next_trajectory_msg.points[point_idx].accelerations.append(point.accelerations[joint_idx])
                if joint_idx < len(point.effort):
                    controller.next_trajectory_msg.points[point_idx].effort.append(point.effort[joint_idx])

        # Publish trajectories and clear
        for c in self.controllers:
            if len(c.next_trajectory_msg.joint_names) > 0:
                c.publisher.publish(c.next_trajectory_msg)

            c.next_trajectory_msg = trajectory_msgs.msg.JointTrajectory()
