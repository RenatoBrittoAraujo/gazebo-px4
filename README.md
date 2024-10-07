# gazebo px4

Para mais intruções de como usar, assista: https://www.youtube.com/watch?v=FEXy4CM2EQs

Os terminais são tmux - um gerenciador de terminal. CTRL+B+SETA permite se mover de terminal pra terminal no tmux.

# Instale

```
git clone https://github.com/RenatoBrittoAraujo/gazebo-px4.git
```

```
cd passos
./1_install_gazebo_PX4.sh
./2_install_ros_ubuntu_2204.sh
./3_install_MicroXRCEAgent.sh
./4_install_deps.sh
./5_install_tmux.sh
./6_install_qgroundcontrol.sh
```

sudo apt install ros-humble-ros-gzgarden

# Rode

Em um terminal
```
./run_nocam.sh
```

Em outro terminal
```
./QGroundControl.AppImage
```

# Info

Para usar o terminal **tmux**, digite CTRL+B+SETA para direção que você quer mudar o terminal atual.

Compilar pela primeira vez
`colcon build`

Compilar e rodar código novo
`colcon build --packages-select fase_1; ros2 run fase_1 fase1_script`

### Instale bibliotecas 

Na pasta `ws/src`

```
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
- NOTE QUAL A BRANCH DO PX4_MSGS, [LEIA](https://github.com/PX4/px4_msgs?tab=readme-ov-file)
- INSTALEI MANUALMENTE `pip install packaging==22.0`
- `pip install lark-parser`
- `pip3 install --user empy==3.3.4`
<!-- - pip install --upgrade setuptools==70.0.0 -->
- pip install setuptools==70.3.0
- pip install -U catkin_pkg
- pip install opencv-python
/edra/PX4-Autopilot/Tools/setup/ubuntu.sh
colcon build
```

### px4

cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl

### Containers

pra rodar rapido
`sudo docker update --cpu-shares 0 <container_id>`

# Fontes

Consulte este link para explicação da dependencias: https://px4.gitbook.io/px4-user-guide/robotics/ros/ros2/ros2_comm

Consulte para explicação do código: https://docs.px4.io/main/en/ros2/offboard_control.html

