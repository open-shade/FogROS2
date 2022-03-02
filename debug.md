# Development
##  Git clone and build docker image
```
git clone https://github.com/BerkeleyAutomation/FogROS2
cd FogROS2
pushd . && cd mpt_ros && mkdir external && cd external && git clone https://github.com/UNC-Robotics/mpt.git && git clone https://github.com/UNC-Robotics/nigh.git && popd
docker build -t fogros2:latest .
```

## Docker run
```
# FOGROS_REPO=~/code/derek/FogROS2
FOGROS_REPO=~/FogROS2
docker run -it --rm \
    --net=host --cap-add=NET_ADMIN \
    -v "${FOGROS_REPO}":/home/root/fog_ws/src/fogros2 \
    fogros2 /bin/bash
```
## vscode attach to the container
In vscode, 
```
Menu:View:Command Palette:Remote-containers:Attach to running container
```

Then open /home/root/fog_ws.

Add follwing to remote settings.json (one time)
```
"python.analysis.extraPaths": ["/opt/ros/rolling/lib/python3.8/site-packages"],
```

## Container environment
In container, does:
```
export AWS_ACCESS_KEY_ID=AKIA6BOHFAN5MQFOLLVO
export AWS_SECRET_ACCESS_KEY=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
# export AWS_DEFAULT_REGION=us-east-2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.xml
. /opt/ros/rolling/setup.sh
```

## Change code and Build
```
colcon build --merge-install && source install/setup.bash
```
Build only one package:
```
rm -rf install
colcon build --packages-select mpt_ros && source install/setup.bash
```
## Test
```
source install/setup.bash
rm -f ./build/mpt_ros/CMakeCache.txt # fix cloud node build failure, TODO
ros2 launch mpt_ros mpt.launch.py
```

## Debug VM
In container,
```
cd /tmp/fogros/<VM id>
cat info # to get ip
chmod 400 *.pem
ssh -i <pem> ubuntu@<ip>
```
In VM, 
```
cd ~/fog_ws
source ./install/setup.sh
install/lib/mpt_ros/node
```
In container,
```
source ./install/setup.sh
install/lib/mpt_ros/client
```

## Build docker image for production
After development and test are done, build docker image for production
```
docker build -t fogros2:latest .
```

# Production
```
docker run -it --rm --net=host --cap-add=NET_ADMIN fogros2

export AWS_ACCESS_KEY_ID=AKIA6BOHFAN5MQFOLLVO
export AWS_SECRET_ACCESS_KEY=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
# export AWS_DEFAULT_REGION=us-east-2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/share/fogros2/configs/cyclonedds.xml

. install/setup.bash
rm -f ./build/mpt_ros/CMakeCache.txt # fix cloud node build failure, TODO
colcon build --merge-install && source install/setup.bash
ros2 launch mpt_ros mpt.launch.py
```

# References
- https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

- https://osrf.github.io/ros2multirobotbook/ros2_api.html
- https://thispointer.com/stdbind-tutorial-and-usage-details/
- https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html
- https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node
- https://docs.ros2.org/foxy/api/rclcpp/index.html



- https://tier4.github.io/autoware.proj/tree/main/developer_guide/knowhow/PortingToROS2/

- https://github.com/osrf/rmf_core/blob/6968346f40421d93039d898acdcbc3ca21f9ceeb/rmf_traffic/CMakeLists.txt
- https://github.com/osrf/rmf_core/blob/6968346f40421d93039d898acdcbc3ca21f9ceeb/rmf_traffic/package.xml
- https://github.com/osrf/rmf_core/tree/6968346f40421d93039d898acdcbc3ca21f9ceeb/rmf_dispenser_msgs

- https://medium.com/@danieljeswin/introduction-to-programming-with-ros2-services-77273d7e8ddc

- https://docs.ros.org/en/foxy/Tutorials/Single-Package-Define-And-Use-Interface.html

- https://stackoverflow.com/questions/58085079/import-an-external-library-into-a-ros-node

- https://docs.ros.org/en/rolling/Contributing/Migration-Guide.html
- https://docs.ros.org/en/rolling/Contributing/Migration-Guide-Python.html

- https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html

- https://github.com/osrf/rmf_core/pull/80
- https://github.com/flexible-collision-library/fcl/issues/537

- https://docs.ros.org/en/rolling/Tutorials/Custom-ROS2-Interfaces.html?highlight=srv
- https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Service-And-Client.html?highlight=add_two_ints
- https://automaticaddison.com/how-to-create-a-service-and-client-c-ros2-foxy-fitzroy/
- https://github.com/ros2/demos

- [best practice of vscode remote-containers](https://stelligent.com/2020/05/29/development-acceleration-through-vs-code-remote-containers-how-we-leverage-vs-code-remote-containers-for-rapid-development-of-cfn_nag/) 
- https://code.visualstudio.com/remote/advancedcontainers/avoid-extension-reinstalls
- https://code.visualstudio.com/docs/remote/containers
- https://code.visualstudio.com/docs/remote/create-dev-container

- https://medium.com/@danieljeswin/introduction-to-programming-with-ros2-launch-files-52eac873f9d0
- https://roboticsbackend.com/ros2-launch-file-example/

- http://wiki.ros.org/fcl
- https://picknik.ai/docs/Realtime_Motion_Planning_And_MoveIt_2.pdf

# AWS credential
```
AWSAccessKeyId=AKIA6BOHFAN5MQFOLLVO
AWSSecretKey=vPVpBcidL1PTsGZX7kCGIQUoceTJk4ivxcxdfuAj
```