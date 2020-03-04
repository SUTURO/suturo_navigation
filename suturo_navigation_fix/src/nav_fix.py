#!/usr/bin/env python
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException

class NavfixServer():

    def __init__(self, name):
        self._retires = 3
        self._dist_fac = 0.1
        self._preempt = False
        self._action_server = actionlib.SimpleActionServer(name, MoveBaseAction, execute_cb = self.execute_cb, auto_start = False)
        self._action_server.register_preempt_callback(self.preempt_cb)
        self._action_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)

        self._action_server.start()
        self._action_client.wait_for_server()

    def preempt_cb(self):
        self._preempt = True
        self._action_client.cancel_goal()
        self._action_server.set_preempted(self._action_client.get_result())

    def execute_cb(self, goal):
        self._preempt = False
        tries = 0
        result = None
        while True and not self._preempt:

            self._action_client.send_goal(goal, feedback_cb=self.feedback_cb)
            self._action_client.wait_for_result()
            result = self._action_client.get_result()
            if self._action_client.get_state() == GoalStatus.ABORTED:
                try:
                    hsr_transform = self._tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time())

                    dx = (goal.target_pose.pose.position.x - hsr_transform.transform.translation.x) * self._dist_fac
                    dy = (goal.target_pose.pose.position.y - hsr_transform.transform.translation.y) * self._dist_fac

                    new_goal = MoveBaseGoal()
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = goal.header.frame_id
                    pose.pose.position.x = hsr_transform.transform.translation.x + dx
                    pose.pose.position.y = hsr_transform.transform.translation.y + dy
                    pose.pose.orientation = goal.target_pose.pose.orientation

                    new_goal.target_pose = pose

                    self._action_client.send_goal(new_goal, feedback_cb=self.feedback_cb)
                    self._action_client.wait_for_result()
                    result = self._action_client.get_result()
                except (LookupException, ConnectivityException):
                    print "Exception lookup transform baselink"
                tries += 1
            if self._action_client.get_state() == GoalStatus.SUCCEEDED or self._action_client.get_state() == GoalStatus.PREEMPTED or tries == self._retires:
                break
        if self._action_client.get_state() == GoalStatus.SUCCEEDED:
            self._action_server.set_succeeded(result)
        elif self._action_client.get_state() == GoalStatus.PREEMPTED:
            self._action_server.set_preempted(result)
        else:
            self._action_server.set_aborted(result)


    def feedback_cb(self, feedback):
        self._action_server.publish_feedback(feedback)

if __name__ == '__main__':
    rospy.init_node("nav_fix")
    nfs = NavfixServer(rospy.get_name())
    rospy.spin()