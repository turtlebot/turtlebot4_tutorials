#!/usr/bin/env python3

# Copyright 2023 Clearpath Robotics, Inc.
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
# @author Ryan Gariepy (rgariepy@clearpath.ai)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('openai_api_key', default_value='', description='API key set up via https://platform.openai.com/account/api-keys'),
        DeclareLaunchArgument('model_name', default_value='gpt-3.5-turbo', description='OpenAI model name as selected from https://platform.openai.com/docs/guides/gpt'),
        Node(
            package='turtlebot4_openai_tutorials',
            executable='natural_language_nav',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'openai_api_key': LaunchConfiguration('openai_api_key')},
                {'model_name': LaunchConfiguration('model_name')}
            ]
        ),
    ])