#!/usr/bin/env python3
# 第8章 ROS机器狗拓展课程\4.传感器开发课程\第1课 发光超声波传感器控制(8.ROS Robot Expanded Course\4.Sensor Development Course\Lesson 1 Glowy Ultrasonic Sensor Control)
import os
import sys
import time
import signal
import sensor.Sonar as Sonar
import sensor.MP3 as MP3

print('''
**********************************************************
*******************功能:超声波控制例程(function:ultrasonic control routine)**********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


# 关闭检测函数(close detection function)
run_st = True
def Stop(signum, frame):
    global run_st
    run_st = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

def calculate_color(distance):
    """
    Calculate an RGB color that transitions:
    - Blue to Green (BLUE_TO_GREEN_RANGE)
    - Green to Red (GREEN_TO_RED_RANGE)
    """
    # Define the distance ranges and transition colors
    DISTANCE_RANGE = (100, 500)  # Min and max distances
    BLUE_TO_GREEN_RANGE = (500, 250)  # Transition range for Blue to Green
    GREEN_TO_RED_RANGE = (250, 100)  # Transition range for Green to Red

    # Cap the distance to the defined range
    min_distance, max_distance = DISTANCE_RANGE
    distance = max(min_distance, min(distance, max_distance))

    # Transition from Blue to Green
    if BLUE_TO_GREEN_RANGE[1] <= distance <= BLUE_TO_GREEN_RANGE[0]:
        start, end = BLUE_TO_GREEN_RANGE
        ratio = (start - distance) / (start - end)  # Normalize to [0, 1]
        red_intensity = 0
        green_intensity = int(255 * ratio)
        blue_intensity = int(255 * (1 - ratio))

    # Transition from Green to Red
    elif GREEN_TO_RED_RANGE[1] <= distance <= GREEN_TO_RED_RANGE[0]:
        start, end = GREEN_TO_RED_RANGE
        ratio = (start - distance) / (start - end)  # Normalize to [0, 1]
        red_intensity = int(255 * ratio)
        green_intensity = int(255 * (1 - ratio))
        blue_intensity = 0

    return (red_intensity, green_intensity, blue_intensity)

if __name__ == '__main__':

    s = Sonar.Sonar()
    s.setRGBMode(0)  # 0:彩灯模块,1:呼吸灯模式(0:color light module, 1:breathing light mode)
    s.setRGB(1, (0, 0, 0))  # 关闭RGB灯(turn off RGB light)
    s.setRGB(0, (0, 0, 0))

    addr = 0x7b #传感器iic地址(sensor I2C address)
    mp3 = MP3.MP3(addr)
    mp3.pause()

    while run_st:
        time.sleep(0.1)
        distance = s.getDistance()  # 获得检测的距离(obtain detected distance)
        print('distance: {}(mm)'.format(distance))
        
        # Calculate the gradient color based on distance
        color = calculate_color(distance)
        
        # Set the RGB LEDs to the calculated color
        s.setRGB(1, color)
        s.setRGB(0, color)
        
    # Turn off RGB lights when stopping
    s.setRGB(1, (0, 0, 0))
    s.setRGB(0, (0, 0, 0))
