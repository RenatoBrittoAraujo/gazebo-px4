# Dev environment PX4-Autopilot

Usa-se:
- PX4-Autopilot
- Gazebo
- ROS2
- Seu workspace com código de drone

## Como instalar esse sistema?

#### 0. Procure as dependencias

Para instalar tmux:
```
sudo apt install tmux -y
```

Para instalar Docker Compose:
```
sudo apt-get install docker-compose-plugin
```

Para instalar QGroundControl:
```
./passos/6_install_qgroundcontrol.sh
```

*se acontecer algum problema no download do QGroundControl, busque o download no google*


#### 1. Clone esse repositório
```
git clone https://github.com/RenatoBrittoAraujo/gazebo-px4
``` 

#### 2. (Opcional) Clone seu workspace

Se você possui algum **workspace customizado** com código para seu drone:
```
rm -rf ws
git clone <meu workspace> ws --recursive
```

#### 3. Baixe a imagem docker

- Baixe a imagem docker a partir de plataforma dockerhub
- Esta imagem **já foi construida de ponta a ponta** e não requer nenhuma configuração adicional da sua parte 
- Ela pesa **~21GB**, **VAI DEMORAR**

Rode: 
```
sudo docker image pull renatobrittoaraujo/px4-autopilot-v1:latest
```

#### 4. Inicie a imagem docker

*Lembre-se de remover container chamado `ros-px4-humble` antes de rodar isso* 

O container está configurado para usar **12GB** de RAM (no máximo) e **4 CPUS**. Se isso não for possível no seu setup, basta abrir o `docker-compose.yml` e alterar. É bem simples de fazer.


O docker-compose.yml usa sua placa de vídeo por meio de uma interface fornecida pelo ubuntu. **Se você não está usando ubuntu, não tem como garantir que vai dar certo.** Nesse caso, edite o docker-compose.yml para adaptar ao linux.

```
sudo docker compose up -d 
```

## Como rodar meu código?

#### Opção 1: Levantar tudo automaticamente
Se você quiser iniciar todos os terminais necessários automaticamente, rode: 

```
./run_docker.sh
```

Isso usa o gerenciador de terminais **tmux**

Para trocar entre terminais tmux, digite CTRL+B+SETA para direção que você quer mudar o terminal atual.

Lembre-se de colocar sua senha do sistema em todos os terminais.

#### Opção 2: Rodar terminais individualmente


**Para abrir o container, roda**
```
sudo docker exec -it ros-px4-humble /bin/bash
```

A única pasta que deveria te interesssar no container é a pasta `/edra`. Todo o resto pode ignorar.

As intruções a seguir vão utilizar o comando de abrir o container apresentado.

**Rodar o STIL: gazebo pelo PX4-Autopilot** 
```
sudo docker exec -it ros-px4-humble /bin/bash -c "cd PX4-Autopilot && make px4_sitl gz_x500_depth"
```

**Rodar o seu código no workspace**
```
sudo docker exec -it ros-px4-humble /bin/bash -c "cd ws; source /opt/ros/humble/setup.bash; source install/setup.bash; colcon build --packages-select fase_3; ros2 run fase_3 fase3_script"
```

**Rodar o sistema de mensageria que permite ROS se comunicar com Gazebo**
```
sudo docker exec -it ros-px4-humble /bin/bash -c "MicroXRCEAgent udp4 -p 8888 "
```

**Rodar a controle de solo QGroundControl**
```
./QGroundControl.AppImage
```

**Rodar o sistema que vai disponibiliza imagens da camera do gazebo para o ROS**
```
sudo docker exec -it ros-px4-humble /bin/bash -c "cd ws; source install/setup.bash; ros2 run ros_gz_image image_bridge /camera"
```

## Fontes

Consulte este link para explicação da dependencias: https://px4.gitbook.io/px4-user-guide/robotics/ros/ros2/ros2_comm

Consulte para explicação do código: https://docs.px4.io/main/en/ros2/offboard_control.html

