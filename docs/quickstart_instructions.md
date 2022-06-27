---
layout: page
title: Quickstart Instructions
permalink: /quickstart_instructions
nav_order: 4
---

Quickstart Instructions
===
This is a quick start guide for installing FogROS 2 (and ROS 2) and its requisites from scratch (e.g., in a VM). New contributors to the project can start here. You can also watch our video tutorials here: 
<iframe width="560" height="315" src="https://www.youtube.com/embed/IfR0JjOytuE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

and 

<iframe width="560" height="315" src="https://www.youtube.com/embed/tXH0kxx7LqU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


1. Install Ubuntu 20.04 or Ubuntu 22.04. See [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) for a tutorial. 

2. Upgrade
```bash
sudo apt update
sudo apt upgrade
```

3. Reboot
```bash
reboot
```

4. Get UTF-8 locale installed
```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

5. Setup sources for ROS 2 (https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)
```bash
sudo apt update
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

6. Install ROS 2 Packages (https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)
```bash
sudo apt update
sudo apt install ros-rolling-desktop
```

7. Add env to startup
```bash
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
source /opt/ros/rolling/setup.bash
```

8. Choose and set a `ROS_DOMAIN_ID` (in range 0 to 101) (https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html)
```bash
export ROS_DOMAIN_ID=99
echo 'export ROS_DOMAIN_ID=99' >> ~/.bashrc
```

9. Install colcon and git
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install git
```

10. Create a workspace
```bash
mkdir -p ~/fog_ws/src
```

11. Clone
```bash
cd ~/fog_ws/src
git clone -b humble https://github.com/BerkeleyAutomation/FogROS2.git
cp FogROS2/fogros2/configs/cyclonedds.ubuntu.$(lsb_release -rs | sed 's/\.//').xml ../cyclonedds.xml
```

12. Build
```bash
#If you see a warning like this, you are fine “On Ubuntu 22.04 this may generate deprecation warnings.  These may be ignored.”
cd ~/fog_ws
colcon build
```

13. Install AWS CLI
```bash
sudo apt install awscli
```

14. Configure AWS Basic Settings. To run the next command, you need to have your [security credentials, an output format and AWS Region.](https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-quickstart.html)
```bash
aws configure
```

15. Install additional dependencies
```bash
sudo apt install python3-pip wireguard
pip install boto3 paramiko scp wgconfig
```

16. If using Ubuntu 22.04
```bash
sudo apt install ros-rolling-rmw-cyclonedds-cpp
```
   
17. Run basic example. Note that the last command may take some time to complete especially the first time it is run. If your setup is correct, you should see the talker node publishing
                                                                                                                                                                     
```bash
cd ~/fog_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
export CYCLONEDDS_URI=file://$(pwd)/install/fogros2/share/fogros2/configs/cyclonedds.ubuntu.$(lsb_release -rs | sed 's/\.//').xml

ros2 launch fogros2_examples talker.aws.launch.py
```

18. You are done. Refer to other relevant github pages including  [README](https://github.com/BerkeleyAutomation/FogROS2/blob/main/README.md) for additional information including [Command Line Interface commands](https://berkeleyautomation.github.io/FogROS2/cli), which allow you do a lot with your cloud instances from the command line, and [Docker installation](https://github.com/BerkeleyAutomation/FogROS2#docker) for those who want to use FogROS2 in a Docker environment.

Next we’ll terminate the demo by typing CTRL-C twice. The first one terminates the robot node, the second one terminates the cloud node.

We can see the cloud computer that FogROS 2 launched for us using the FogROS command-line interface or CLI. “ros2 fog list” shows that we have one running instance with the name XXX. Finally, we terminate the instance so that we are no longer being charged for it by running “ros2 fog delete XXX”. You can verify that it is being deleted and by running “ros2 fog list” again. Observe the “status shutting down” line. After a short while, running ros2 fog list will show nothing, indicating that the instance is terminated and you are no longer being charged.
                                                  
```bash
#To see the name of a FogROS2 instance
ros2 fog list

#To delete a FogROS2 instance
ros2 fog delete [name]

Typing CTRL-C kills the local instance (e.g., listener) the first time and then the cloud instance the second time
```
19. Our video tutorials for Docker installation and native installations are include below.

Docker:
<iframe width="560" height="315" src="https://www.youtube.com/embed/oEnmZXojkcI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Native Installation:
<iframe width="560" height="315" src="https://www.youtube.com/embed/JlV4DhArb8Q" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

