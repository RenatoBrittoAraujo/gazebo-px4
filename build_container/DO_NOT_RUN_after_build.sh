pip install setuptools==70.3.0
sudo apt install ros-humble-cv-bridge
pip install "numpy<2"
sudo apt install ros-humble-ros-gzgarden -y

apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \

sudo apt install --reinstall -y \
 ros-humble-desktop \
 ros-dev-tools \
 ros-humble-ros-gzgarden 

sudo apt-get install ros-humble-ros-ign-bridge

sudo apt remove ros-humble-ros-gzgarden-bridge ros-humble-ros-gzgarden-interfaces

sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces

rm $(grep -r "packages.ros.org/ros2/ubuntu" /etc/apt/*)
rm $(ls /usr/share/keyrings | grep "ros")

sudo rm /etc/apt/sources.list.d/ros2.list
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /etc/apt/sources.list.d/ros-latest.list
sudo rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
rm -rf /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list
sudo apt remove --purge 'ros-*' -y
sudo apt autoremove -y
sudo rm -rf /opt/ros/
sudo rm -rf ~/.ros
sudo rm -rf ~/ros2_ws
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /etc/apt/sources.list.d/ros-latest.list
sudo rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt clean
sudo rm -rf /var/lib/apt/lists/*


sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


git clone -b release/1.15 https://github.com/PX4/PX4-Autopilot --recursive

sudo apt install ros-humble-ros-gzgarden
apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y

sudo apt install -y ros-humble-desktop 
sudo apt install -y ros-dev-tools 
sudo apt install -y ros-humble-ros-gzgarden

sudo apt-get install ros-humble-ros-ign-bridge
