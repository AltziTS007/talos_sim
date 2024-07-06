#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from tf import transformations

'''
Created: 27/06/2024

@author: Altzi Tsanko
email: ace.tsan21@gmail.com
'''

class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    ># waypoint     Pose2D      Target waypoint for navigation.
    ># flag_start   bool        Flag to indicate the start of navigation.

    <= arrived                  Navigation to target pose succeeded.
    <= failed                   Navigation to target pose failed.
    """

    def __init__(self):
        """Constructor"""

        super(MoveBaseState, self).__init__(outcomes=['arrived', 'failed'],
                                            input_keys=['waypoint'],
                                            output_keys=['flag_start'])

        self._action_topic = "/move_base"
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._arrived = False
        self._failed = False

    def execute(self, userdata):
        """
        Wait for action result and return outcome accordingly.

        :param userdata: FlexBE user data
        :return: Outcome of the state
        """
        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                userdata.flag_start = False
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'

    def on_enter(self, userdata):
        """
        Create and send action goal when entering the state.

        :param userdata: FlexBE user data
        """
        self._arrived = False
        self._failed = False

        # Create and populate action goal
        goal = MoveBaseGoal()
        pt = Point(x=userdata.waypoint.x, y=userdata.waypoint.y)
        qt = transformations.quaternion_from_euler(0, 0, userdata.waypoint.theta)
        goal.target_pose.pose = Pose(position=pt, orientation=Quaternion(*qt))
        goal.target_pose.header.frame_id = "map"

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True

    def cancel_active_goals(self):
        """Cancel any active goals."""
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    def on_exit(self, userdata):
        """
        Cancel active goals when exiting the state.

        :param userdata: FlexBE user data
        """
        self.cancel_active_goals()

    def on_stop(self):
        """Cancel active goals when stopping the state."""
        self.cancel_active_goals()
