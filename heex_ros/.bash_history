ls
ll
roscore
rqt
rqt-graph
roscore
exit
rosversion -d
roscore
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
roscore
code .
exit
rosbag play example_offline_extraction.bag
roscore
rosbag play example_offline_extraction.bag
ls
cd Heex_SDK_2_37_1/
ls
rosbag play example_offline_extraction.bag
cd samples/
ls
cd ROS1
ls
cd ../../..
ls
rosbag play example_offline_extraction.bag
lsb -a
lsb_relese -a
lsb_release -a
uname -m
cd Heex_SDK_2_37_1/samples/ROS1/ros-edge/
catkin_make -DHEEX_SDK_DIR=~/Heex_SDK_2_37_1
ls ~/Heex_SDK_2_37_1/sdk/CustomerSide
catkin_make -DHEEX_SDK_DIR=$HOME/Heex_SDK_2_37_1
ls
rm -rf build devel install
ls
catkin_make -DHEEX_SDK_DIR=$HOME/Heex_SDK_2_37_1
rostopic list
rviz
rostopic list
rostopic echo /detection_image 
rostopic echo /imu
ls
cd home/
ls
cd abhimanyu/
cd Heex_SDK_2_39_1/samples/ROS1/ros-edge/
catkin_make -DHEEX_SDK_DIR=$HOME/Heex_SDK_2_39_1
cd Heex_SDK_2_39_1/
./runHeexKernel.sh
code .
wget -qO - https://apt.packages.shiftkey.dev/gpg.key | gpg --dearmor | sudo tee /usr/share/keyrings/shiftkey-packages.gpg > /dev/null
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/shiftkey-packages.gpg] https://apt.packages.shiftkey.dev/ubuntu/ any main" > /etc/apt/sources.list.d/shiftkey-packages.list'
wget -qO - https://apt.packages.shiftkey.dev/gpg.key | gpg --dearmor | sudo tee /usr/share/keyrings/shiftkey-packages.gpg > /dev/null
apt-get update
apt-get install ca-certificates
sudo apt-get install ca-certificates
wget -qO - https://apt.packages.shiftkey.dev/gpg.key | gpg --dearmor | sudo tee /usr/share/keyrings/shiftkey-packages.gpg > /dev/null
(type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) && sudo mkdir -p -m 755 /etc/apt/keyrings && wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null && sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null && sudo apt update && sudo apt install gh -y
sudo (type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) \
&& sudo mkdir -p -m 755 /etc/apt/keyrings \
&& wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
gh auth login
sudo (type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) \
&& sudo mkdir -p -m 755 /etc/apt/keyrings \
&& wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh -y
sudo (type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) \
sudo (type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) 
(type -p wget >/dev/null || (sudo apt update && sudo apt-get install wget -y)) && sudo mkdir -p -m 755 /etc/apt/keyrings && wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null && sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null && sudo apt update && sudo apt install gh -y
deb [trusted=yes] https://apt.packages.shiftkey.dev/ubuntu
exit
