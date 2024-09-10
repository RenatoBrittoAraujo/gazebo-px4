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


# Fontes

Consulte este link para explicação da dependencias: https://px4.gitbook.io/px4-user-guide/robotics/ros/ros2/ros2_comm

Consulte para explicação do código: https://docs.px4.io/main/en/ros2/offboard_control.html

