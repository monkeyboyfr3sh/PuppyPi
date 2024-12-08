#!/usr/bin/python3
#coding=utf8
# 控制蜂鸣器播放多个频率 (Control buzzer to play multiple frequencies)

import rospy
from ros_robot_controller.msg import BuzzerState

# 控制蜂鸣器 (Control Buzzer)

print('''
**********************************************************
*************** 功能: 蜂鸣器多频率控制例程 (Buzzer Multi-Frequency Control Routine) ***************
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
Online mall: https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下 Ctrl+C 可关闭此次程序运行，若失败请多次尝试！(Press Ctrl+C to close this program, please try multiple times if it fails)
----------------------------------------------------------
''')

def play_frequencies(buzzer_pub, frequencies):
    for freq, on_time, off_time, repeat in frequencies:
        buzzer_msg = BuzzerState()
        buzzer_msg.freq = freq
        buzzer_msg.on_time = on_time
        buzzer_msg.off_time = off_time
        buzzer_msg.repeat = repeat
        
        print(f"Playing frequency: {freq} Hz for {on_time} sec (repeat: {repeat})")
        buzzer_pub.publish(buzzer_msg)
        rospy.sleep((on_time + off_time) * repeat + 0.5)  # Adding a buffer between sounds

if __name__ == '__main__':
    try:
        # 初始化节点 (Initialize Node)
        rospy.init_node('buzzer_control_demo')
        
        buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
        rospy.sleep(0.5)  # 延时 (Delay)

        # 定义要播放的频率列表 (List of frequencies to play: [frequency, on_time, off_time, repeat])
        frequencies = [
            (1000, 1, 0.5, 2),  # 1000 Hz for 1 sec, 0.5 sec off, repeated 2 times
            (1500, 0.5, 0.5, 3),  # 1500 Hz for 0.5 sec, 0.5 sec off, repeated 3 times
            (2000, 2, 1, 1),  # 2000 Hz for 2 sec, 1 sec off, repeated once
            (2500, 1, 0.2, 4)  # 2500 Hz for 1 sec, 0.2 sec off, repeated 4 times
        ]

        # 播放频率 (Play the frequencies)
        play_frequencies(buzzer_pub, frequencies)

        print("Buzzer sequence complete!")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")
