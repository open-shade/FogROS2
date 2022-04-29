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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Talker example that launches everything on the robot (compare to talker.ubuntu.*.launch.py)
    """
    ld = LaunchDescription()

    talker_node = Node(package="fogros2_examples", executable="listener", output="screen")
    listener_node = Node(
        package="fogros2_examples", executable="talker", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld