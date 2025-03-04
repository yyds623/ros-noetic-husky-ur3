

husky-UR3
=========

  ✨ 实现了对移动单臂机器人的Gzebo物体搬运仿真模拟Demo演示，并实现利用DWA+RRT结合算法，对路径进行规划与底盘建图定位控制。考虑到可以使用视觉信息进行物体的识别与抓取但是由于时间原因并未实现，之后有时间再更新吧，如果对你有所帮助请不要吝啬🌟🌟🌟呀。✨        
                                                                                     -->3<--
  
[![复合移动机械臂在gazebo仿真下的展示](https://i0.hdslb.com/bfs/archive/c53ce528a40641a3aca32c366f7bb01fc716764f.jpg@672w_378h_1c.avif)](https://www.bilibili.com/video/BV1RAyTYFEPP/?share_source=copy_web&vd_source=4194ae4a17aa95105a0ab346c7c66fbb)

### 1、安装ros-noetic

------------------------------------------
首先安装ros-noetic系统，按照以下命令配置即可

First, install the ros-noetic system and configure it according to the following commands.
```bash
wget http://fishros.com/install -O fishros && . fishros  #ros one-click installation
```
------------------------------------------
### 2、安装代码库


------------------------------------------

```bash
mkdir -p ~/ros_ws/src  # Creating a ros workspace
cd ~/ros_ws/src/ && git clone --recurse-submodules https://github.com/yyds623/ros-noetic-husky-ur3.git  #克隆代码仓

cp ./src/models.zip ~/.gazebo && cd ~/.gazebo/&& unzip models.zip #导入Gazebo模型文件


rosdepc install --from-path src --ignore-src -r -y  #安装基本依赖项
cd ~/ros_ws && catkin_make   # 编译
```
------------------------------------------

### 3、启动gazebo


```bash

#新开一个终端
source ~/ros_ws/devel/setup.bash 
roslaunch husky_ur3_gazebo husky_ur3_HRI_lab.launch  #启动gazebo


#新开一个终端
source ~/ros_ws/devel/setup.bash 
roslaunch husky_ur3_gripper_moveit_config Omni_control.launch    # 启动 MoveIt & RViz  运动


#新开一个终端
source ~/ros_ws/devel/setup.bash 
roslaunch husky_ur3_navigation husky_ur3_in_HRI_lab_amcl.launch  #定位
```
------------------------------------------
### 4、执行底盘路径点导航与机械臂运动及其夹爪控制
```bash
#新开一个终端
source ~/ros_ws/devel/setup.bash && roscd husky_ur3_navigation/src && python nf.py   #导航时间与机械臂到 front_view 的时间  并导航 根据提示输入相应位置的数字
#新开一个终端
source ~/ros_ws/devel/setup.bash && roscd husky_ur3_navigation/src && python fg.py   #计算底盘路径长度
```
------------------------------------------

### 5、其他小Tips
### Tip-1
```bash

# 夹爪打开
rostopic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 0.0"

# 夹爪关闭
rostopic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 0.65"
```
### Tip-2
```bash
rospack plugins --attrib=plugin nav_core  #查询插件move_base自定义算法
```

![reconfigure 调整参数](https://github.com/user-attachments/assets/4b61d531-fe5b-4327-ae23-0a44e94fd983)

### Tip-3
```bash
rosrun rqt_reconfigure rqt_reconfigure  #调整参数
```
![查询插件move_base自定义算法](https://github.com/user-attachments/assets/5f163e80-e4bd-4aa8-b3f7-1a3ca72a2d84)


### Tip-4
```bash
rosrun moveit_commander moveit_commander_cmdline.py  #查看机械臂的关节角度
```
![查看机械臂的关节角度](https://github.com/user-attachments/assets/aaf9debe-05ed-4142-81c9-dce46c95810a)
```
