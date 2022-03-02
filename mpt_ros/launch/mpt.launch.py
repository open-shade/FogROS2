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
    machine1 = fogros2.AWS(region="us-west-1", ec2_instance_type="c5d.24xlarge", ami_image="ami-0f3a19bd1ec82ca18") # ami-09175f2ca3c3dc67c, t2.medium

    client_node = Node(
        package="mpt_ros", executable="client", output="screen")
    node_node = fogros2.CloudNode(
        package="mpt_ros", executable="node", output="screen",
        machine = machine1)
    ld.add_action(client_node)
    ld.add_action(node_node)
    return ld
