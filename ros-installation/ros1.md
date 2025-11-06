<center><font color=blue face='宋体' size=8>安装Ros1</font></center>

# 一、 下载ubuntu 20.04 版本
## 建议使用镜像来下载：
- [清华源链接]：https://mirrors.tuna.tsinghua.edu.cn/
 > 1.点击后在搜索栏搜索ubuntu，点击ubuntu-releases，选择相应的版本号20.04，再点击ubuntu-20.04.6-desktop-amd64.iso，点击后电脑会自行 下载。        
 > 2.在VMware Workstation Pro中点击添加新的虚拟机。        
 > 3.点击光盘映像文件，选择刚刚下载好的ubuntu20.04的相应文件。后续的设置基本上都不需要修改。        
 > 4.安装完成后，需要点击左下角九个点的图标，及显示所有应用，找到Software & Updates 的应用并点击，在Ubuntu Software下找到 Download from，在后面找到others，并选择 China ，再选择一个镜像网站，以更改下载源。
 # 二、 下载ros1
 - [网址]https://wiki.ros.org/cn

 > 点击安装，选择最右面那幅图上的 ROS Noetic Ninjemys。再点击ubuntu,然后按照从上到下去执行就可以了。

 #### 注意 : 设置sources.list时建议点击 ***ROS镜像源*** ，用里面的国内的镜像源。

 > 如果你不想找，直接复制下面的一些指令（在终端中运行）：
 >> 1. ``` sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'```        
 >> 2. `sudo apt update`         
 >> #### 如果有 failed 字样显示，要进行下面操作：
 >> 输入 `sudo nano /etc/apt/sources.list` , 将里面的内容全部删除。     
 >> 然后将 
 ``` deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse

deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse

deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse

deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse  
```
>
>> 复制并粘贴进去。再按 ctrl+o 保存，点击回车，再按 ctrl+x 退出。       
>> 3. ` sudo apt install ros-noetic-desktop-full`            
>> 4. ` echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc `
### 进行上述步骤后基本安装好了，可以进行下述操作进行验证：
- 打开终端输入`roscore`
- 按下 ctrl+Alt+t 打开新的终端，再输入 `rosrun turtlesim turtlesim_node`,会弹出一个带有小乌龟的小窗。
- 按下 ctrl+Alt+t 打开新的终端，再输入`rosrun turtlesim turtle`,最后输入`rosrun turtlesim turtle_teleop_key`
- 此时点击键盘的上下左右键可以控制乌龟移动。
