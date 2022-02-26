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

from launch import FogROSLaunchDescription
from launch_ros.actions import Node
import fogros2

def generate_launch_description():
    ld = FogROSLaunchDescription()

    import os
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/root/fog_ws/gcp.json"

    machine1 = fogros2.GCP(project_id = "canvas-rampart-342500", project_number = "662998256920")

    talker_node = Node(
        package="fogros2_examples", executable="listener", output="screen")
    listener_node = fogros2.CloudNode(
        package="fogros2_examples", executable="talker", output="screen",
        machine = machine1)
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld