#!/usr/bin/env python3

import py_trees
import rclpy.qos
from std_msgs.msg import Bool

from py_trees_ros import subscribers

class BallTOBlackBoard(subscribers.ToBlackboard):

    def __init__(self,
                 name: str,
                 topic_name: str,
                 qos_profile: rclpy.qos.QoSProfile,
                 status: bool=False):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=Bool,
                         qos_profile=qos_profile,
                         blackboard_variables={"ball": None},
                         clearing_policy=py_trees.common.ClearingPolicy.NEVER
                         )
        self.blackboard.register_key(
            key="is_ball_detected",
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.ball = Bool()
        self.blackboard.is_ball_detected = False   # decision making
        self.status = status

    def update(self):

        status = super(BallTOBlackBoard, self).update()

        if status != py_trees.common.Status.RUNNING:
            self.blackboard.is_ball_detected = False
        return status
