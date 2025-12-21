
# building for first time
## Install ros2 jazzy
Follow this link:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
## Install Gazebo
```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/gazebo-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | \
sudo tee /etc/apt/sources.list.d/gazebo-stable.list
```

```bash
sudo apt update
sudo apt install gz-harmonic
```

## Install turtle bot
```bash
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-ros-gz
```




# About the project
## System architecture
<img width="541" height="341" alt="System Architecture" src="https://github.com/user-attachments/assets/c5133500-eafe-434f-85a9-78f72944ea8d" />
