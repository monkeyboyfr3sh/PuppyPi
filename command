#关掉所有ros后台
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
#或者
~/.stop_ros.sh

# 第6章 ROS机器狗标准课程\
# 2.ROS机器狗逆运动学控制课程\第2课 逆运动学简要分析
rosrun puppy_control puppy_IK_demo.py

# 4.ROS+OpenCV视觉识别课程\第6课 颜色识别
rosrun puppy_standard_functions color_detect_demo.py

# 4.ROS+OpenCV视觉识别课程\第7课 标签识别
rosrun puppy_standard_functions apriltag_detect_demo.py

# 4.ROS+OpenCV视觉识别课程\第8课 AR视觉
rosrun puppy_standard_functions apriltag_AR_demo.py

# 5.ROS+OpenCV视觉追踪课程\第1课 色块坐标定位实验
rosrun puppy_standard_functions color_tracking_demo.py

# 5.ROS+OpenCV视觉追踪课程\第2课 颜色追踪实验
rosrun puppy_standard_functions color_tracking_demo.py

# 5.ROS+OpenCV视觉追踪课程\第3课 标签坐标定位实验
rosrun puppy_standard_functions apriltag_tracking_demo.py

# 5.ROS+OpenCV视觉追踪课程\第4课 标签追踪实验
rosrun puppy_standard_functions apriltag_tracking_demo.py


# 第7章 ROS机器狗创意课程
# 1.AI视觉人脸检测\第2课 人脸检测
rosrun puppy_advanced_functions face_detect_demo.py

# 2.AI识别台阶攀爬\第1课 台阶识别
rosrun puppy_advanced_functions negotiate_stairs_demo.py

# 2.AI识别台阶攀爬\第2课 自主台阶攀爬
rosrun puppy_advanced_functions negotiate_stairs_demo.py

# 3.AI自主追踪踢球\第1课 小球寻找与定位
rosrun puppy_advanced_functions kick_ball_demo.py

# 3.AI自主追踪踢球\第2课 自主追踪踢球
rosrun puppy_advanced_functions kick_ball_demo.py

# 4.AI视觉巡线行走\第1课 线条定位
rosrun puppy_advanced_functions visual_patrol_demo.py

# 4.AI视觉巡线行走\第2课 自主巡线行走
rosrun puppy_advanced_functions visual_patrol_demo.py

# 5.机身自平衡\第1课 ROS机器人姿态检测
rosrun puppy_advanced_functions self_balancing_demo.py

# 5.机身自平衡\第2课 ROS机器人姿态自平衡
rosrun puppy_advanced_functions self_balancing_demo.py


# 第8章 ROS机器狗拓展课程
# 2.树莓派扩展板课程\第2课 控制RGB彩灯
rosrun puppy_extend_demo rgb_control_demo.py

# 2.树莓派扩展板课程\第3课 控制蜂鸣器
rosrun puppy_extend_demo buzzer_control_demo.py

# 2.树莓派扩展板课程\第4课 按键控制
# 使用该玩法前需要再docker外面打开LX终端关闭按键服务
# 否则会冲突，使用完之后再重新开启按键服务
sudo systemctl stop button_scan.service # 关闭按键服务
rosrun puppy_extend_demo button_control_demo.py
sudo systemctl start button_scan.service# 开启按键服务

# 2.树莓派扩展板课程\第5课 姿态控制
rosrun puppy_extend_demo posture_detect_demo.py

# 3.空心杯舵机控制课程\第2课 控制空心杯舵机转动
rosrun puppy_extend_demo servo_control_single.py

# 3.空心杯舵机控制课程\第3课 控制舵机速度
rosrun puppy_extend_demo servo_control_speed.py

# 4.传感器开发课程\第1课 发光超声波传感器控制
rosrun puppy_extend_demo sonar_control_demo.py

# 4.传感器开发课程\第2课 机器狗超声波测距避障
rosrun puppy_extend_demo sonar_avoidance.py

# 4.传感器开发课程\第3课 触摸传感器检测
rosrun puppy_extend_demo touch_detect_demo.py

# 4.传感器开发课程\第4课 点阵模块显示
rosrun puppy_extend_demo lattice_display_demo.py

# 4.传感器开发课程\第5课 语音识别传感器实验
rosrun puppy_extend_demo ASR_detect_demo

# 4.传感器开发课程\第6课 机器狗触摸检测感应
rosrun puppy_extend_demo touch_control_demo.py

# 4.传感器开发课程\第7课 MP3模块实验
rosrun puppy_extend_demo mp3_moonwalk_demo.py

# 4.传感器开发课程\第8课 机器狗语音识别交互
rosrun puppy_extend_demo voice_interaction_demo.py

# 第10章 SLAM建图与导航课程（PuppyPi Pro专属）
# 相关操作根据教程文档
# 1.二维SLAM地图构建\第2课 Gmapping建图算法
roslaunch puppy_slam gmapping.launch
rosparam set /puppy_control/joint_state_pub_topic true
roslaunch puppy_description rviz_with_urdf.launch
rosrun map_server map_saver -f /home/ubuntu/puppypi/src/puppy_slam/maps/map1

# 1.二维SLAM地图构建\第3课 Hector建图算法
roslaunch puppy_slam hector.launch
rosparam set /puppy_control/joint_state_pub_topic true
roslaunch puppy_description rviz_with_urdf.launch
rosrun map_server map_saver -f /home/ubuntu/puppypi/src/puppy_slam/maps/map1

# 1.二维SLAM地图构建\第4课 Karto建图算法
roslaunch puppy_slam karto.launch
rosparam set /puppy_control/joint_state_pub_topic true
roslaunch puppy_description rviz_with_urdf.launch
rosrun map_server map_saver -f /home/ubuntu/puppypi/src/puppy_slam/maps/map1

# 2.SLAM自主导航\第3课 雷达单点导航与避障
roslaunch puppy_navigation navigation.launch
rosparam set /puppy_control/joint_state_pub_topic true
roslaunch puppy_description rviz_with_urdf.launch


# 第11章 ROS机器狗结合机械臂课程
# 第3课 颜色识别夹取
~/.stop_ros.sh
roslaunch puppy_bringup start_node_with_arm.launch
roslaunch puppy_with_arm color_detect_with_arm.launch

# 第4课 自主识别夹取
~/.stop_ros.sh
roslaunch puppy_with_arm color_grab.launch

# 第5课 巡线夹取
~/.stop_ros.sh
roslaunch puppy_with_arm visual_patrol_with_arm.launch

# 第6课 手势控制机械臂
# 回传画面打开浏览器输入192.168.149.1:8080，选择/hand_control/image_result
~/.stop_ros.sh
roslaunch puppy_with_arm hand_control_with_arm.launch

# 第8课 定点导航搬运
# 相关操作根据对应教程文档
# PuppyPi内
~/.stop_ros.sh
roslaunch puppy_pi_bringup start_node_nav.launch
roslaunch puppy_navigation navigation.launch
rosrun puppy_with_arm color_place_nav.py
# 虚拟机内
rosparam set /puppy_control/joint_state_pub_topic true
roslaunch puppy_description rviz_with_urdf.launch

