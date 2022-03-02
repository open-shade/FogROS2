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
    # machine1 = fogros2.AWS(region="us-west-1", ec2_instance_type="t2.medium", ami_image="ami-09175f2ca3c3dc67c")

    client_node = Node(
        package="mpt_ros", executable="client", output="screen")
    node_node = Node(
        package="mpt_ros", executable="node", output="screen")
    ld.add_action(client_node)
    ld.add_action(node_node)
    return ld
