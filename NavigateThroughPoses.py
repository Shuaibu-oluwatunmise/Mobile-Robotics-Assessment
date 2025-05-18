#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.1256, -0.0387, 0.0880], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([1.7129, 3.4903, 0.0024], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([2.5819, -1.9250, -0.0014], TurtleBot4Directions.NORTH))
    goal_pose.append(navigator.getPoseStamped([-2.3577, 1.9174, -0.0014], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([-2.1933, 3.9677, -0.0014], TurtleBot4Directions.WEST))


    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
