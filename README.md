# Proyecto_Igus_Rebel
## 1. Instalar ROS2 

`sudo add-apt-repository universe`

`sudo apt update && sudo apt install curl -y`

`sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`

`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`

`sudo apt update && sudo apt install ros-dev-tools`

`sudo apt upgrade`  

`sudo apt install ros-jazzy-desktop` 

`source /opt/ros/jazzy/setup.bash`  

## 2. Extras

`sudo apt install mesa-utils`  
 
`sudo apt install python3-pip`  
    
`sudo apt install jstest-gtk`   
    
`sudo apt install ros-jazzy-joy*`  
    
`sudo apt install ros-jazzy-joint-state-publisher`  
    
`sudo apt install ros-jazzy-joint-state-publisher-gui`  
    
`sudo apt install ros-jazzy-ros2-control`  
    
`sudo apt install ros-jazzy-ros2-controllers`  
    
`sudo apt install ros-jazzy-ros-gz`  
    
`sudo apt install ros-jazzy-gz-ros2-control `  
    
`sudo apt install ros-jazzy-navigation2`  
    
`sudo apt install ros-jazzy-nav2-bringup`  

## 3. Instalar numpy

`pip install numpy`

#### 3.1 Si no instala

`python3 -m venv ~/mi_venv`

`source ~/mi_venv/bin/activate`

`pip install numpy`

## 4. Crear el repositorio

`mkdir <Nombre repo>`

`cd <Nombre repo>`

`mkdir src`

`colcon build`

`ros2 pkg create my_package --build-type ament_python --node-name <Nombre nodo>`
    
    

