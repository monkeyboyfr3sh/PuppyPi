#!/usr/bin/python3
#coding=utf8
# Control buzzer to play a simple Christmas tune

import sys
import rospy
from ros_robot_controller.msg import BuzzerState
from enum import Enum

# Control Buzzer

print('''
**********************************************************
*************** Function: Buzzer Christmas Tune Control ***************
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
Online mall: https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to stop this program. If it fails, please try multiple times!
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
    """Enumeration for note durations as fractions of a whole note."""
    WHOLE = 1.0       # Whole note
    HALF = 0.5        # Half note
    QUARTER = 0.25    # Quarter note
    EIGHTH = 0.125    # Eighth note
    SIXTEENTH = 0.0625 # Sixteenth note

# Set BPM
BPM = 300  # Adjust this value to control the tempo (e.g., 100 for slower, 150 for faster)

# Calculate the duration of a whole note in seconds based on BPM
WHOLE_NOTE_DURATION = 60 / BPM * 4  # 4 beats per whole note

# "Jingle Bells" note list ([note, duration])
JINGLE_BELLS = [
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.HALF),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.HALF),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.G5, Duration.QUARTER), (NoteFreq.C5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.E5, Duration.WHOLE),
    (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER),
    (NoteFreq.F5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.D5, Duration.HALF), (NoteFreq.G5, Duration.HALF)
]

def play_tune(buzzer_pub, tune, whole_note_duration):
    for note, duration in tune:
        note_duration = whole_note_duration * duration.value
        buzzer_msg = BuzzerState()
        buzzer_msg.freq = note.value
        buzzer_msg.on_time = note_duration
        buzzer_msg.off_time = 0.1  # Short pause between notes
        buzzer_msg.repeat = 1

        if note == NoteFreq.REST:
            print(f"Rest for {duration.name.lower()} note (duration: {note_duration:.2f} sec)")
        else:
            print(f"Playing note: {note.name} ({note.value} Hz) for {duration.name.lower()} note (duration: {note_duration:.2f} sec)")

        buzzer_pub.publish(buzzer_msg)
        rospy.sleep(note_duration + buzzer_msg.off_time)  # Wait for note duration + pause

if __name__ == '__main__':
    try:
        # Initialize Node
        rospy.init_node('buzzer_christmas_tune')

        buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
        rospy.sleep(0.5)  # Delay

        # Play "Jingle Bells" tune
        play_tune(buzzer_pub, JINGLE_BELLS, WHOLE_NOTE_DURATION)

        print("Jingle Bells tune complete!")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")
        sys.exit(0)
