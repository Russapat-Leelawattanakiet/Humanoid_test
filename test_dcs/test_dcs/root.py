#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros.trees
import py_trees.console as console
from py_trees_ros import subscribers
import ball2bb
import rclpy
import sys


def create_root():

    root = py_trees.composites.Parallel(
        name = "Tutorial_root", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    ballToBB = ball2bb.BallTOBlackBoard(
        name="Ball2BB",
        topic_name="/ball/detection",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        status=False
    )
    
    root.add_child(topics2bb)
    topics2bb.add_child(ballToBB)
    
    return root
    
def tutorial_main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="ball", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

tutorial_main()