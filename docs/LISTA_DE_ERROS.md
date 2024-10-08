### root@renato:/edra/ws# colcon build
Starting >>> px4_msgs
[Processing: px4_msgs]                            
--- stderr: px4_msgs                               
Traceback (most recent call last):
  File "/edra/ws/build/px4_msgs/ament_cmake_python/px4_msgs/setup.py", line 4, in <module>
    setup(
  File "/root/.local/lib/python3.10/site-packages/setuptools/__init__.py", line 117, in setup
    return distutils.core.setup(**attrs)
  File "/root/.local/lib/python3.10/site-packages/setuptools/_distutils/core.py", line 183, in setup
    return run_commands(dist)
  File "/root/.local/lib/python3.10/site-packages/setuptools/_distutils/core.py", line 199, in run_commands
    dist.run_commands()
  File "/root/.local/lib/python3.10/site-packages/setuptools/_distutils/dist.py", line 954, in run_commands
    self.run_command(cmd)
  File "/root/.local/lib/python3.10/site-packages/setuptools/dist.py", line 950, in run_command
    super().run_command(command)
  File "/root/.local/lib/python3.10/site-packages/setuptools/_distutils/dist.py", line 973, in run_command
    cmd_obj.run()
  File "/root/.local/lib/python3.10/site-packages/setuptools/command/egg_info.py", line 311, in run
    self.find_sources()
  File "/root/.local/lib/python3.10/site-packages/setuptools/command/egg_info.py", line 319, in find_sources
    mm.run()
  File "/root/.local/lib/python3.10/site-packages/setuptools/command/egg_info.py", line 545, in run
    self.prune_file_list()
  File "/root/.local/lib/python3.10/site-packages/setuptools/command/sdist.py", line 161, in prune_file_list
    super().prune_file_list()
  File "/root/.local/lib/python3.10/site-packages/setuptools/_distutils/command/sdist.py", line 380, in prune_file_list
    base_dir = self.distribution.get_fullname()
  File "/root/.local/lib/python3.10/site-packages/setuptools/_core_metadata.py", line 267, in get_fullname
    return _distribution_fullname(self.get_name(), self.get_version())
  File "/root/.local/lib/python3.10/site-packages/setuptools/_core_metadata.py", line 285, in _distribution_fullname
    canonicalize_version(version, strip_trailing_zero=False),
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
gmake[2]: *** [CMakeFiles/ament_cmake_python_build_px4_msgs_egg.dir/build.make:70: CMakeFiles/ament_cmake_python_build_px4_msgs_egg] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:447: CMakeFiles/ament_cmake_python_build_px4_msgs_egg.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< px4_msgs [36.4s, exited with code 2]
                                 
Summary: 0 packages finished [36.5s]
  1 package failed: px4_msgs
  1 package had stderr output: px4_msgs
  5 packages not processed
root@renato:/edra/ws# 


SOLUCAO
pip install setuptools==70.3.0

### root@renato:/edra/ws# source /opt/ros/humble/setup.bash; source install/setup.bash; colcon build --packages-select fase_3; ros2 run fase_3 fase3_script
Starting >>> fase_3  
Finished <<< fase_3 [1.05s]          

Summary: 1 package finished [1.15s]
Traceback (most recent call last):
  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 33, in <module>
    sys.exit(load_entry_point('fase-3==0.0.0', 'console_scripts', 'fase3_script')())
  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 883, in exec_module
  File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
  File "/edra/ws/install/fase_3/lib/python3.10/site-packages/fase_3/fase3_script.py", line 21, in <module>
    from cv_bridge import CvBridge
ModuleNotFoundError: No module named 'cv_bridge'
[ros2run]: Process exited with failure 1


SOLUÇÃO:
sudo apt install ros-humble-cv-bridge

### root@renato:/edra/ws# source /opt/ros/humble/setup.bash; source install/setup.bash; colcon build --packages-select fase_3; ros2 run fase_3 fase3_script
Starting >>> fase_3  
Finished <<< fase_3 [0.97s]          

Summary: 1 package finished [1.08s]

A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.1.1 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.
Some module may need to rebuild instead e.g. with 'pybind11>=2.12'.

If you are a user of the module, the easiest solution will be to
downgrade to 'numpy<2' or try to upgrade the affected module.
We expect that some modules will need time to support NumPy 2.

Traceback (most recent call last):  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 33, in <module>
    sys.exit(load_entry_point('fase-3==0.0.0', 'console_scripts', 'fase3_script')())
  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "/edra/ws/install/fase_3/lib/python3.10/site-packages/fase_3/fase3_script.py", line 21, in <module>
    from cv_bridge import CvBridge
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/cv_bridge/__init__.py", line 6, in <module>
    from cv_bridge.boost.cv_bridge_boost import cvtColorForDisplay, getCvType
AttributeError: _ARRAY_API not found

A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.1.1 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.
Some module may need to rebuild instead e.g. with 'pybind11>=2.12'.

If you are a user of the module, the easiest solution will be to
downgrade to 'numpy<2' or try to upgrade the affected module.
We expect that some modules will need time to support NumPy 2.

Traceback (most recent call last):  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 33, in <module>
    sys.exit(load_entry_point('fase-3==0.0.0', 'console_scripts', 'fase3_script')())
  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "/edra/ws/install/fase_3/lib/python3.10/site-packages/fase_3/fase3_script.py", line 22, in <module>
    import cv2
AttributeError: _ARRAY_API not found
Traceback (most recent call last):
  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 33, in <module>
    sys.exit(load_entry_point('fase-3==0.0.0', 'console_scripts', 'fase3_script')())
  File "/edra/ws/install/fase_3/lib/fase_3/fase3_script", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 883, in exec_module
  File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
  File "/edra/ws/install/fase_3/lib/python3.10/site-packages/fase_3/fase3_script.py", line 22, in <module>
    import cv2
ImportError: numpy.core.multiarray failed to import
[ros2run]: Process exited with failure 1


SOLUÇÃO:
pip install "numpy<2"


### O DRONE NAO SE MEXE, COMUNICACAO INVALIDA

SOLUÇÃO:
sudo apt install ros-humble-ros-gzgarden -y

### O DRONE NAO SE MEXE, COMUNICACAO INVALIDA

SOLUÇÃO:
reinstallar todas as outras coisas


apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \

### O DRONE NAO SE MEXE, COMUNICACAO INVALIDA

Nenhum nó encontrado do gazebo em ros2 topic list

SOLUÇÃO:
sudo apt install --reinstall -y \
 ros-humble-desktop \
 ros-dev-tools \
 ros-humble-ros-gzgarden 

### root@renato:/edra/ws# sudo apt-get install ros-humble-ros-ign-bridge

Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 ros-humble-ros-gzgarden-bridge : Conflicts: ros-humble-ros-gz-bridge but 0.244.15-1jammy.20240901.074810 is to be installed
 ros-humble-ros-gzgarden-interfaces : Conflicts: ros-humble-ros-gz-interfaces but 0.244.15-1jammy.20240728.220333 is to be installed
E: Error, pkgProblemResolver::Resolve generated breaks, this may be caused by held packages.

*sudo apt list ros-humble-ros-gzgarden-\**
Listing... Done
ros-humble-ros-gzgarden-bridge/unknown,now 0.244.11-1002jammy amd64 [installed,automatic]
ros-humble-ros-gzgarden-image/unknown,now 0.244.11-1002jammy amd64 [installed,automatic]
ros-humble-ros-gzgarden-interfaces/unknown,now 0.244.11-1002jammy amd64 [installed,automatic]
ros-humble-ros-gzgarden-sim-demos/unknown,now 0.244.11-1002jammy amd64 [installed,automatic]
ros-humble-ros-gzgarden-sim/unknown,now 0.244.11-1002jammy amd64 [installed,automatic]

*sudo apt list ros-humble-ros-gzgarden-\**
Listing... Done
ros-humble-ros-gz-bridge-dbgsym/jammy 0.244.15-1jammy.20240901.074810 amd64
ros-humble-ros-gz-bridge/jammy 0.244.15-1jammy.20240901.074810 amd64
ros-humble-ros-gz-image-dbgsym/jammy 0.244.15-1jammy.20240901.084038 amd64
ros-humble-ros-gz-image/jammy 0.244.15-1jammy.20240901.084038 amd64
ros-humble-ros-gz-interfaces-dbgsym/jammy 0.244.15-1jammy.20240728.220333 amd64
ros-humble-ros-gz-interfaces/jammy 0.244.15-1jammy.20240728.220333 amd64
ros-humble-ros-gz-sim-dbgsym/jammy 0.244.15-1jammy.20240729.003859 amd64
ros-humble-ros-gz-sim-demos/jammy 0.244.15-1jammy.20240901.084325 amd64
ros-humble-ros-gz-sim/jammy 0.244.15-1jammy.20240729.003859 amd64

- sudo apt remove ros-humble-ros-gzgarden-bridge ros-humble-ros-gzgarden-interfaces

- sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces

### SEM TOPICO GAZEBO CRIADO POR PX4


test find old to delete: `grep -r "packages.ros.org/ros2/ubuntu" /etc/apt/*`
`ls /usr/share/keyrings | grep "ros"`

sudo apt remove \
 ros-humble-desktop \
 ros-dev-tools \
 ros-humble-ros-gzgarden 
sudo apt autoremove

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


### SEM TOPICO GAZEBO CRIADO POR PX4

SOLUÇÃO: rebuildar PX4 do zero na branch correta
`git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot --recursive`
`make px4_sitl gz_x500_depth`

### make px4_sitl gz_x500_depth

WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data

SOLUCAO:
sudo apt install ros-humble-ros-gzgarden
apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base

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

### VERSAO DO GAZEBO BEM ANTIGA, ATUALIZARR

sudo apt install -y ros-humble-desktop 
sudo apt install -y ros-dev-tools 
sudo apt install -y ros-humble-ros-gzgarden

export ROS_DISTRO=jazzy


python package not found: symforce


INSTALLING ROS BRIDGE WITH: sudo apt-get install ros-humble-ros-ign-bridge

### FUNCIONOU!!!!!!!!!!!!!!!!!!

mas tem o error 

2024-10-07 22:03:12.527 [RTPS_READER_HISTORY Error] Change payload size of '204' bytes is larger than the history payload size of '191' bytes and cannot be resized. -> Function can_change_be_added_nts

verifique que nenhum outro processo esta rodando para porta 8888
`sudo lsof -i:8888`

Pode ser q você está rodando `MicroXRCEAgent udp4 -p 8888` em cima da porta que o gazebo roda
P

