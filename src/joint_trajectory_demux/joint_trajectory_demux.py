import rospy
import trajectory_msgs.msg
import controller_manager_msgs.srv._ListControllers

LIST_CONTROLLERS_SRV_NAME = "/controller_manager/list_controllers"


class Controller:
    def __init__(self):
        self.name = ""
        self.topic = ""
        self.joints = []
        self.publisher = None
        self.next_trajectory_msg = trajectory_msgs.msg.JointTrajectory()


def is_active(controller):
    return controller.state == "running"


def is_joint_trajectory_controller(controller):
    return "JointTrajectoryController" in controller.type


def collect_joints(controller):
    joints = []
    for claimed in controller.claimed_resources:
        joints += claimed.resources
    return joints


class JointTrajectoryDemux:
    def __init__(self, ns, pns):
        self.ns = ns
        self.pns = pns
        self.controllers = []

        self.trajectory_sub = rospy.Subscriber("~command", trajectory_msgs.msg.JointTrajectory, self._trajectory_cb)

    def discover_controllers(self):
        rospy.logdebug("Discovering controllers")
        # see http://wiki.ros.org/ROS/Master_API
        master = rospy.get_master()
        system_state = master.getSystemState()
        cm_services = [service for service, _provider in system_state[2][2] if LIST_CONTROLLERS_SRV_NAME in service]
        for service in cm_services:
            list_controllers = rospy.ServiceProxy(service, controller_manager_msgs.srv.ListControllers)
            try:
                resp = list_controllers()
            except rospy.ServiceException as exc:
                rospy.logwarn("Service did not process request: " + str(exc))
            else:
                for controller in resp.controller:
                    if is_joint_trajectory_controller(controller) and is_active(controller):
                        c = Controller()
                        ns = service[:service.find(LIST_CONTROLLERS_SRV_NAME)]
                        c.name = ns + "/" + controller.name
                        c.topic = c.name + "/command"
                        c.publisher = rospy.Publisher(c.topic, trajectory_msgs.msg.JointTrajectory, queue_size=1)
                        c.joints = collect_joints(controller)
                        rospy.logdebug("Added controller '" + c.name + ".")
                        rospy.logdebug("--- topic: " + c.topic)
                        rospy.logdebug("--- joints: " + ",".join(c.joints))
                        self.controllers.append(c)

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

    def _trajectory_cb(self, trajectory_msg):
        if len(self.controllers) == 0:
            rospy.logwarn("Controllers have not been discovered yet.")
            return
        # Fill next trajectory message of each controller with same trajectory points
        for c in self.controllers:
            c.next_trajectory_msg.header = trajectory_msg.header
            for point in trajectory_msg.points:
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
