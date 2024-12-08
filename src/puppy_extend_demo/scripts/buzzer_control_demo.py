#!/usr/bin/python3
#coding=utf8
# 控制蜂鸣器播放简单圣诞歌曲 (Control buzzer to play a simple Christmas tune)

import rospy
from ros_robot_controller.msg import BuzzerState
from enum import Enum

# 控制蜂鸣器 (Control Buzzer)

print('''
**********************************************************
*************** 功能: 蜂鸣器播放圣诞歌曲 (Buzzer Christmas Tune Control) ***************
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
Online mall: https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下 Ctrl+C 可关闭此次程序运行，若失败请多次尝试！(Press Ctrl+C to close this program, please try multiple times if it fails)
----------------------------------------------------------
''')

class NoteFreq(Enum):
    """Enumeration for musical note frequencies."""
    B4 = 494
    C5 = 523
    D5 = 587
    E5 = 659
    F5 = 698
    G5 = 784
    A5 = 880
    REST = 0

class Duration(Enum):
    """Enumeration for note durations in seconds."""
    WHOLE = 2.0
    HALF = 1.0
    QUARTER = 0.5
    EIGHTH = 0.25
    SIXTEENTH = 0.125

# 设置节奏倍速 (Set tempo multiplier: < 1 for faster, > 1 for slower)
TEMPO = 0.5  # Reduce this value to make the tempo faster

# "Jingle Bells" 音符列表 ([note, duration])
JINGLE_BELLS = [
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.HALF),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.HALF),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.G5, Duration.QUARTER), (NoteFreq.C5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.E5, Duration.WHOLE),
    (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER),
    (NoteFreq.F5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.D5, Duration.HALF), (NoteFreq.G5, Duration.HALF)
]

def play_tune(buzzer_pub, tune, tempo):
    for note, duration in tune:
        buzzer_msg = BuzzerState()
        buzzer_msg.freq = note.value
        buzzer_msg.on_time = duration.value * tempo
        buzzer_msg.off_time = 0.1 * tempo  # Short pause between notes
        buzzer_msg.repeat = 1

        if note == NoteFreq.REST:
            print(f"Rest for {duration.name.lower()} note (adjusted duration: {buzzer_msg.on_time:.2f} sec)")
        else:
            print(f"Playing note: {note.name} ({note.value} Hz) for {duration.name.lower()} note (adjusted duration: {buzzer_msg.on_time:.2f} sec)")

        buzzer_pub.publish(buzzer_msg)
        rospy.sleep(buzzer_msg.on_time + buzzer_msg.off_time)  # Wait for note duration + pause

if __name__ == '__main__':
    try:
        # 初始化节点 (Initialize Node)
        rospy.init_node('buzzer_christmas_tune')

        buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
        rospy.sleep(0.5)  # 延时 (Delay)

        # 播放 "Jingle Bells" (Play Jingle Bells tune)
        play_tune(buzzer_pub, JINGLE_BELLS, TEMPO)

        print("Jingle Bells tune complete!")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")
