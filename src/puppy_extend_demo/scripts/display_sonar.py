#!/usr/bin/env python3
import os
import sys
import time
import signal
from collections import deque
from sensor import dot_matrix_sensor
import sensor.Sonar as Sonar
import sensor.MP3 as MP3

# Ensure Python 3 is used
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
********* 功能: 超声波传感器测距与点阵显示结合例程 *********
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
Online mall: https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
   (Press Ctrl+C to close this program, please try multiple times if it fails)
----------------------------------------------------------
''')

# Initialize Dot Matrix Sensor
dms = dot_matrix_sensor.TM1640(dio=7, clk=8)

# Initialize Sonar Sensor
sonar = Sonar.Sonar()

# Initialize MP3 Module
addr = 0x7b  # Sensor I2C address
mp3 = MP3.MP3(addr)
mp3.loopOff()
mp3.volume(30)
mp3.next()
mp3.play()
mp3.pause()

# Handle Program Stop
run_st = True
def Stop(signum, frame):
    global run_st
    run_st = False
    print('关闭中...')
    dms.clear()
    sonar.setRGB(1, (0, 0, 0))
    sonar.setRGB(0, (0, 0, 0))

signal.signal(signal.SIGINT, Stop)

# Character mapping for fixed messages
char_map = {
    'A': [0x3E, 0x09, 0x09, 0x3E, 0x00],
    'B': [0x3F, 0x25, 0x25, 0x1A, 0x00],
    'C': [0x1E, 0x21, 0x21, 0x12, 0x00],
    'D': [0x3F, 0x21, 0x21, 0x1E, 0x00],
    'E': [0x3F, 0x25, 0x25, 0x21, 0x00],
    'F': [0x3F, 0x05, 0x05, 0x01, 0x00],
    'G': [0x1E, 0x21, 0x29, 0x3A, 0x00],
    'H': [0x3F, 0x04, 0x04, 0x3F, 0x00],
    'I': [0x21, 0x3F, 0x21, 0x00, 0x00],
    'J': [0x10, 0x20, 0x21, 0x1F, 0x00],
    'K': [0x3F, 0x04, 0x0A, 0x31, 0x00],
    'L': [0x3F, 0x20, 0x20, 0x20, 0x00],
    'M': [0x3F, 0x02, 0x04, 0x02, 0x3F],
    'N': [0x3F, 0x02, 0x04, 0x3F, 0x00],
    'O': [0x1E, 0x21, 0x21, 0x1E, 0x00],
    'P': [0x3F, 0x09, 0x09, 0x06, 0x00],
    'Q': [0x1E, 0x21, 0x11, 0x2E, 0x00],
    'R': [0x3F, 0x09, 0x19, 0x26, 0x00],
    'S': [0x26, 0x25, 0x25, 0x19, 0x00],
    'T': [0x01, 0x01, 0x3F, 0x01, 0x01],
    'U': [0x1F, 0x20, 0x20, 0x1F, 0x00],
    'V': [0x0F, 0x10, 0x20, 0x10, 0x0F],
    'W': [0x3F, 0x10, 0x08, 0x10, 0x3F],
    'X': [0x31, 0x0A, 0x04, 0x0A, 0x31],
    'Y': [0x03, 0x04, 0x38, 0x04, 0x03],
    'Z': [0x31, 0x29, 0x25, 0x23, 0x00],
    '0': [0x1E, 0x29, 0x25, 0x1E, 0x00],
    '1': [0x22, 0x3F, 0x20, 0x00, 0x00],
    '2': [0x32, 0x29, 0x29, 0x26, 0x00],
    '3': [0x12, 0x21, 0x25, 0x1A, 0x00],
    '4': [0x0C, 0x0A, 0x3F, 0x08, 0x00],
    '5': [0x17, 0x25, 0x25, 0x19, 0x00],
    '6': [0x1E, 0x25, 0x25, 0x18, 0x00],
    '7': [0x01, 0x39, 0x05, 0x03, 0x00],
    '8': [0x1A, 0x25, 0x25, 0x1A, 0x00],
    '9': [0x06, 0x29, 0x29, 0x1E, 0x00],
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
    '!': [0x00, 0x2F, 0x00, 0x00, 0x00],
    '.': [0x20, 0x00, 0x00, 0x00, 0x00],
    '-': [0x08, 0x08, 0x08, 0x00, 0x00],
    '_': [0x20, 0x20, 0x20, 0x00, 0x00]
}
# Function to display a message on the dot matrix
def display_message(message):
    """Display a fixed message on the dot matrix."""
    # Create the display buffer
    display_buf = []
    for char in message:
        display_buf.extend(char_map.get(char, char_map[' ']))
        display_buf.append(0x00)  # Space between characters
    
    # Ensure the buffer is exactly 16 columns
    display_buf = display_buf[:16] + [0x00] * (16 - len(display_buf))
    
    # Update the dot matrix display
    dms.display_buf = display_buf
    dms.update_display()

# Function to calculate the gradient color based on distance and brightness
def calculate_color(distance, brightness=1.0):
    """
    Calculate an RGB color that transitions:
    - Blue to Green (BLUE_TO_GREEN_RANGE)
    - Green to Red (GREEN_TO_RED_RANGE)
    
    Args:
        distance (int): The measured distance in millimeters.
        brightness (float): A scaling factor between 0.0 (off) and 1.0 (full brightness).
    """
    # Define the distance ranges and transition colors
    DISTANCE_RANGE = (100, 500)  # Min and max distances
    BLUE_TO_GREEN_RANGE = (500, 250)  # Transition range for Blue to Green
    GREEN_TO_RED_RANGE = (250, 100)  # Transition range for Green to Red

    # Cap the distance to the defined range
    min_distance, max_distance = DISTANCE_RANGE
    distance = max(min_distance, min(distance, max_distance))

    # Ensure brightness is within the range [0.0, 1.0]
    brightness = max(0.0, min(1.0, brightness))

    # Transition from Blue to Green
    if BLUE_TO_GREEN_RANGE[1] <= distance <= BLUE_TO_GREEN_RANGE[0]:
        start, end = BLUE_TO_GREEN_RANGE
        ratio = (start - distance) / (start - end)  # Normalize to [0, 1]
        red_intensity = 0
        green_intensity = int(255 * ratio * brightness)
        blue_intensity = int(255 * (1 - ratio) * brightness)

    # Transition from Green to Red
    elif GREEN_TO_RED_RANGE[1] <= distance <= GREEN_TO_RED_RANGE[0]:
        start, end = GREEN_TO_RED_RANGE
        ratio = (start - distance) / (start - end)  # Normalize to [0, 1]
        red_intensity = int(255 * ratio * brightness)
        green_intensity = int(255 * (1 - ratio) * brightness)
        blue_intensity = 0

    return (red_intensity, green_intensity, blue_intensity)

# Maintain a deque to store the last 10 distance samples
distance_samples = deque(maxlen=5)

if __name__ == '__main__':
    try:
        dms.brightness(1)
        sonar.setRGBMode(0)  # Set RGB mode to color light module
        while run_st:
            # Get distance from the sonar sensor
            distance = sonar.getDistance()
            print(f"Distance: {distance} mm")
            
            # Add the new sample to the deque
            distance_samples.append(distance)
            
            # Calculate the average of the last 10 samples
            avg_distance = sum(distance_samples) // len(distance_samples)
            print(f"Averaged Distance: {avg_distance} mm")
            
            # Determine message based on averaged distance
            if avg_distance > 300:
                message = "FAR"
            elif 150 < avg_distance <= 300:
                message = "MID"
            else:
                message = "NEAR"
            
            # Display the message on the dot matrix
            display_message(message)
            
            # Calculate and set the gradient color based on averaged distance
            color = calculate_color(avg_distance, 0.1)
            sonar.setRGB(1, color)
            sonar.setRGB(0, color)
            
            time.sleep(0.005)  # Update every 0.5 seconds

    except KeyboardInterrupt:
        dms.clear()
        sonar.setRGB(1, (0, 0, 0))
        sonar.setRGB(0, (0, 0, 0))
        print("\nDisplay cleared. Exiting program.")
