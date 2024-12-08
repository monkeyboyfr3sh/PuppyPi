#!/usr/bin/env python3
import threading
import os
import sys
import time
import signal
from collections import deque
from sensor import dot_matrix_sensor
import sensor.Sonar as Sonar
import sensor.MP3 as MP3

# Initialize global variables
run_st = True

# Handle program stop
def Stop(signum, frame):
    global run_st
    run_st = False
    print('Stopping...')
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

def calculate_color(distance, brightness=1.0):
    """
    Calculate an RGB color that transitions:
    - Blue to Green to Red, with Green peaking at the midpoint.
    
    Args:
        distance (int): The measured distance in millimeters.
        brightness (float): A scaling factor between 0.0 (off) and 1.0 (full brightness).
    """
    # Define the distance range
    DISTANCE_RANGE = (200, 400)  # Min and max distances
    min_distance, max_distance = DISTANCE_RANGE

    # Calculate the midpoint where green is maximized
    midpoint = (min_distance + max_distance) // 2

    # Cap the distance to the defined range
    distance = max(min_distance, min(distance, max_distance))

    # Ensure brightness is within the range [0.0, 1.0]
    brightness = max(0.0, min(1.0, brightness))

    # Transition from Red to Green (Min to Midpoint)
    if distance <= midpoint:
        ratio = (distance - min_distance) / (midpoint - min_distance)  # Normalize to [0, 1]
        red_intensity = int(255 * (1 - ratio) * brightness)
        green_intensity = int(255 * ratio * brightness)
        blue_intensity = 0

    # Transition from Green to Blue (Midpoint to Max)
    else:
        ratio = (distance - midpoint) / (max_distance - midpoint)  # Normalize to [0, 1]
        red_intensity = 0
        green_intensity = int(255 * (1 - ratio) * brightness)
        blue_intensity = int(255 * ratio * brightness)

    return (red_intensity, green_intensity, blue_intensity)

class MP3Player(MP3.MP3):
    def __init__(self, address=0x7b):
        super().__init__(address)
        self.loopOff()
        self.volume(15)
        # self.pause()

if __name__ == '__main__':
    distance_samples = deque(maxlen=10)
    
    try:
        # Initialize MP3 Module
        player = MP3Player()
        
        # Initialize hardware
        dms = dot_matrix_sensor.TM1640(dio=7, clk=8)
        sonar = Sonar.Sonar()

        # Timer for controlling display update frequency
        display_update_interval = 0.1  # Update display every 0.1 seconds
        last_display_update_time = time.time()

        # Timer for MP3 playback
        playback_duration = 2.0  # Play for 1 second
        playback_start_time = None
        is_playing = False

        dms.brightness(2)
        sonar.setRGBMode(0)  # Set RGB mode to color light module

        while run_st:
            # Get distance from the sonar sensor
            distance = sonar.getDistance()
            
            # Add the new sample to the deque
            distance_samples.append(distance)
            
            # Calculate the average of the last samples
            avg_distance = sum(distance_samples) // len(distance_samples)
            
            # Check if the distance goes below 200 and start playback timer
            if avg_distance < 200 and not is_playing:
                is_playing = True
                player.play()
                playback_start_time = time.time()

            # Stop playback after 1 second
            if is_playing and (time.time() - playback_start_time >= playback_duration):
                player.pause()
                is_playing = False
                playback_start_time = None

            # Check if it's time to update the display
            current_time = time.time()
            if current_time - last_display_update_time >= display_update_interval:
                last_display_update_time = current_time

                # Format the averaged distance as DDD with zero-padding
                distance_str = f"{avg_distance:03d}"

                # Display the formatted distance on the dot matrix
                display_message(distance_str)

                # Calculate and set the gradient color based on averaged distance
                color = calculate_color(avg_distance, 0.2)
                sonar.setRGB(1, color)
                sonar.setRGB(0, color)

            time.sleep(0.001)  # Small delay for loop iteration

    except Exception as e:
        # Spam the off signals for 0.5 seconds
        end_time = time.time() + 1
        while time.time() < end_time:
            dms.clear()
            sonar.setRGB(0, (0, 0, 0))
            sonar.setRGB(1, (0, 0, 0))
            time.sleep(0.01)  # Send off signals every 10 milliseconds