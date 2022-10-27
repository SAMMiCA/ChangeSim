# DIY Data Collection
**change detection dataset collection implemented in Airsim + Unreal Engine + ROS environment.**

 The procedure is summarized in the below figure:
![fig_main](fig/3d-cd-figure.png)

## Environment Summary

 - Ubuntu 20.04 
 - ROS Noetic
 - virtualenv with Python 3.8+
 - [Airsim](https://github.com/microsoft/AirSim) (included in this repo.)
 - UnrealEngine 4.25 (If you want to use UE4.26 or higher, you need to pull the latest official airsim repo)
 - Pytorch 1.7.0+
 
 (may also work in Ubuntu 18.04 with ROS Melodic, but you need to modify installation scrip by yourself.)
 
 # Setup
 
  **0. Setup your virtualenv and install python packages**
   - Create Python 3.8 env.
     ``` 
     pip3 install virtualenv
     python -m virtualenv venv
     virtualenv venv --python=python3.8
     source venv/bin/activate
     pip install future torch torchvision
     ```
    
 **1. Install ROS Noetic**
  - [official installation link](http://wiki.ros.org/noetic/Installation/Ubuntu) 
  - Trouble-shooting:
    - gcc error:
    
    ```
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install gcc-8 g++-8
    gcc-8 --version
    ```
    - Cmake version error:
    
      Download and build latest version of Cmake [here](https://snowdeer.github.io/linux/2018/04/10/upgrade-cmake/https://cmake.org/download/).
   
 
 **2. Install Unreal Engine 4(UE4) on Linux**
  - Asumming that you have installed NVIDIA graphics driver and cuda toolkit 10.2, please follow the instructions below.
  - [official installation link](https://docs.unrealengine.com/ko/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html) 
    (You are required to register for an Epic Games Account.)
    ```
    # go to the folder where you clone GitHub projects
    git clone -b 4.25 https://github.com/EpicGames/UnrealEngine.git
    cd UnrealEngine
    ./Setup.sh
    ./GenerateProjectFiles.sh
    make
    
    sudo apt -y install vulkan-utils
    ```
  - Run UE4(it takes about 15 minutes for the first run):
    ```
    ./UnrealEngine/Engine/Binaries/Linux/UE4Editor
    ```
    
 
 **3. Install Airsim**
  - [official installation link](https://microsoft.github.io/AirSim/build_linux/#pre-build-setup) 
    ```
    # Build Airsim
    git submodule update --init --recursive
    # Before the build, change external source code for fixing the computer vision mode velocity issue
    cp backup/ManualPoseController.cpp external/AirSim/Unreal/Plugins/AirSim/Source/.
    cd external/AirSim
    ./setup.sh
    ./build.sh
    cd PythonClient
    pip install .
    cd ..
    
    # Install ROS with python prerequisites
    sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-mavros*
    sudo apt-get install python3-catkin-tools
    sudo apt-get install ros-noetic-ros-numpy
    # Build ROS packages
    cd ros;
    catkin_make
    
    # Running (just for the installation check)
    source devel/setup.bash;
    roslaunch airsim_ros_pkgs airsim_node.launch;
    roslaunch airsim_ros_pkgs rviz.launch;
    ```
  - Trouble-shooting:
    - gcc error:
    ```
    catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
    ```
    - AttributeError: 'module' object has no attribute 'Interpreter'
    ```
    pip uninstall em
    pip install empy
    ```
    
    - python syntax error
    ```
    pip install git+https://github.com/catkin/catkin_tools.git
    ```
    
    
 **4. Setup your unreal environment**
   - We do not recommend setting a new UE4 project folder (that method sometimes causes a lot of errors). 
     Instead, it is recommended to use the already set Blocks folder.
     (See [here](https://microsoft.github.io/AirSim/unreal_custenv/) if you want to set up a new UE4 project folder.)
   - Use unreal env. from [Electric-Tunnel-Dataset](https://github.com/SAMMiCA/Scenario1-ElectricTunnel-Dataset).
     ```
     # Duplicate Blocks Env as My_Env
     cp -r Airsim/Unreal/Environments/Blocks Airsim/Unreal/Environments/My_Env
     ```
     Download MAP000X.zip from [Electric-Tunnel-Dataset](https://github.com/SAMMiCA/Scenario1-ElectricTunnel-Dataset) and unzip it.
     
     Then, from the unzipped source code, merge `Contents` folder into `Airsim/Unreal/My_Env/Contents`. 
     
   - (optional) Instead of using [Electric-Tunnel-Dataset](https://github.com/SAMMiCA/Scenario1-ElectricTunnel-Dataset), you can use UE4 maps from [UE4 Marketplace](https://www.unrealengine.com/marketplace/ko/store)(PURCHASE REQUIRED)
   

 
 **6. Install RTABMAP and other prerequisites**
```
sudo apt install ros-noetic-rtabmap* ros-noetic-ros-numpy ros-noetic-perception-pcl python-rospy

sudo cp backup/rgbd_slam_airsim_rviz_config.rviz /opt/ros/noetic/share/rtabmap_ros/launch/config/rgbd.rviz

```
     
   - Trouble-shooting:
     
       - No module named sip
    
         install sip mannually following [here](https://wiki.debianusers.or.kr/index.php?title=PyQt_%EC%84%A4%EC%B9%98%ED%95%98%EA%B8%B0#SIP_.EC.84.A4.EC.B9.98)
       

# Visualization
```
python visualization.py
```
<p align="center"><img src="../fig/visualization.png"></p>


# Dataloader



```python
import torch
from dataloader import ChangeSim

# Number of target change detection class
num_class = 5

train_data = ChangeSim(crop_size=(320, 240), set='train', num_classes=num_class)
train_loader = torch.utils.data.DataLoader(train_data, batch_size=6, shuffle=True)
test_data = ChangeSim(crop_size=(320, 240), set='test', num_classes=num_class)
test_loader = torch.utils.data.DataLoader(test_data, batch_size=6, shuffle=False)
```
