#!/usr/bin/python3
#coding=utf8
# Control buzzer to play a simple Christmas tune

import sys
import threading
import queue
import rospy
import sensor.MP3 as MP3
from ros_robot_controller.msg import BuzzerState, RGBState, RGBsState
from enum import Enum

class NoteFreq(Enum):
    """Enumeration for musical note frequencies."""
    C4 = 262
    C_SHARP4 = 277
    D4 = 294
    D_SHARP4 = 311
    E4 = 330
    F4 = 349
    F_SHARP4 = 370
    G4 = 392
    G_SHARP4 = 415
    A4 = 440
    A_SHARP4 = 466
    B4 = 494
    
    C5 = 523
    C_SHARP5 = 554
    D5 = 587
    D_SHARP5 = 622
    E5 = 659
    F5 = 698
    F_SHARP5 = 740
    G5 = 784
    G_SHARP5 = 831
    A5 = 880
    A_SHARP5 = 932
    B5 = 988
    
    REST = 0

class Duration(Enum):
    """Enumeration for note durations as fractions of a whole note."""
    WHOLE = 1.0
    HALF = 0.5
    QUARTER = 0.25
    EIGHTH = 0.125
    SIXTEENTH = 0.0625

class Song:
    """Class to represent a song with name, BPM, and notes."""
    def __init__(self, name, bpm, notes, on_complete=None):
        self.name = name
        self.bpm = bpm
        self.notes = notes
        self.whole_note_duration = 60 / bpm * 4
        self.note_queue = queue.Queue()
        self.on_complete = on_complete

    def queue_notes(self):
        """Queue up all notes to be played."""
        for note, duration in self.notes:
            note_duration = self.whole_note_duration * duration.value
            self.note_queue.put((note, note_duration))

    def play_next_note(self, buzzer_pub, rgb_pub):
        """Play the next note in the queue and sync both LEDs with the note."""
        if not self.note_queue.empty():
            note, note_duration = self.note_queue.get()
            
            # Create the buzzer message
            buzzer_msg = BuzzerState()
            buzzer_msg.freq = note.value
            buzzer_msg.on_time = note_duration
            buzzer_msg.off_time = 0.1
            buzzer_msg.repeat = 1

            # Create the RGB message for both LEDs
            rgb_msg = RGBsState()
            led_1 = RGBState()
            led_1.id = 1

            led_2 = RGBState()
            led_2.id = 2

            if note == NoteFreq.REST:
                # Turn off both LEDs during a rest
                led_1.r, led_1.g, led_1.b = 0, 0, 0
                led_2.r, led_2.g, led_2.b = 0, 0, 0
                print(f"Rest for {note_duration:.2f} sec")
            else:
                # Map note frequencies to colors (simple mapping example)
                led_1.r = (note.value * 3) % 256
                led_1.g = (note.value * 7) % 256
                led_1.b = (note.value * 5) % 256

                # Set the second LED to the same color as the first LED
                led_2.r = led_1.r
                led_2.g = led_1.g
                led_2.b = led_1.b

                print(f"Playing note: {note.name} ({note.value} Hz) for {note_duration:.2f} sec")

            # Add both LEDs to the RGB message
            rgb_msg.data = [led_1, led_2]

            # Publish the buzzer and RGB messages
            buzzer_pub.publish(buzzer_msg)
            rgb_pub.publish(rgb_msg)

            # Schedule the next note to be played after the current note duration + pause
            threading.Timer(note_duration + buzzer_msg.off_time, self.play_next_note, [buzzer_pub, rgb_pub]).start()
        else:
            print(f"'{self.name}' complete!")
            # Turn off both LEDs when the song is complete
            rgb_msg = RGBsState()
            led_1 = RGBState(id=1, r=0, g=0, b=0)
            led_2 = RGBState(id=2, r=0, g=0, b=0)
            rgb_msg.data = [led_1, led_2]
            rgb_pub.publish(rgb_msg)

            if self.on_complete:
                self.on_complete()

    def play(self, buzzer_pub, rgb_pub):
        """Start playing the song asynchronously."""
        print(f"Starting '{self.name}' at {self.bpm} BPM")
        self.queue_notes()
        self.play_next_note(buzzer_pub, rgb_pub)

def play_songs_in_sequence(buzzer_pub, rgb_pub, songs):
    """Play all songs in sequence and exit when done."""
    def play_next():
        if songs:
            current_song = songs.pop(0)
            current_song.on_complete = play_next
            current_song.play(buzzer_pub, rgb_pub)
        else:
            rospy.signal_shutdown("All songs completed.")
            sys.exit(0)

    play_next()

# Define Songs
jingle_bells = Song("Jingle Bells", 300, [
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.HALF),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.HALF),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.G5, Duration.QUARTER), (NoteFreq.C5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.E5, Duration.WHOLE),
    (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER), (NoteFreq.F5, Duration.QUARTER),
    (NoteFreq.F5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER),
    (NoteFreq.E5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.D5, Duration.QUARTER), (NoteFreq.E5, Duration.QUARTER), (NoteFreq.D5, Duration.HALF), (NoteFreq.G5, Duration.HALF)
])

fur_elise = Song("Für Elise", 350, [
    (NoteFreq.E5, Duration.EIGHTH), (NoteFreq.D_SHARP5, Duration.EIGHTH), (NoteFreq.E5, Duration.EIGHTH), (NoteFreq.D_SHARP5, Duration.EIGHTH),
    (NoteFreq.E5, Duration.EIGHTH), (NoteFreq.B4, Duration.EIGHTH), (NoteFreq.D5, Duration.EIGHTH), (NoteFreq.C5, Duration.EIGHTH),
    (NoteFreq.A4, Duration.QUARTER), (NoteFreq.REST, Duration.QUARTER),

    (NoteFreq.C4, Duration.EIGHTH), (NoteFreq.E4, Duration.EIGHTH), (NoteFreq.A4, Duration.EIGHTH), (NoteFreq.B4, Duration.EIGHTH),
    (NoteFreq.REST, Duration.EIGHTH), (NoteFreq.E4, Duration.EIGHTH), (NoteFreq.G_SHARP4, Duration.EIGHTH), (NoteFreq.B4, Duration.EIGHTH),
    (NoteFreq.C5, Duration.QUARTER), (NoteFreq.REST, Duration.QUARTER),

    (NoteFreq.E4, Duration.EIGHTH), (NoteFreq.E5, Duration.EIGHTH), (NoteFreq.D_SHARP5, Duration.EIGHTH), (NoteFreq.E5, Duration.EIGHTH),
    (NoteFreq.D_SHARP5, Duration.EIGHTH), (NoteFreq.E5, Duration.EIGHTH), (NoteFreq.B4, Duration.EIGHTH), (NoteFreq.D5, Duration.EIGHTH),
    (NoteFreq.C5, Duration.EIGHTH), (NoteFreq.A4, Duration.QUARTER),

    (NoteFreq.C4, Duration.EIGHTH), (NoteFreq.E4, Duration.EIGHTH), (NoteFreq.A4, Duration.EIGHTH), (NoteFreq.B4, Duration.EIGHTH),
    (NoteFreq.REST, Duration.EIGHTH), (NoteFreq.E4, Duration.EIGHTH), (NoteFreq.C5, Duration.EIGHTH), (NoteFreq.B4, Duration.EIGHTH),
    (NoteFreq.A4, Duration.QUARTER)
])

if __name__ == '__main__':
    
    addr = 0x7b #传感器iic地址(sensor I2C address)
    mp3 = MP3.MP3(addr)
    mp3.volume(25)
    # mp3.next()
    mp3.play()
    mp3.pause()

    try:
        # # Initialize Node
        rospy.init_node('dancing_dog')

        # buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
        # rgb_pub = rospy.Publisher("/ros_robot_controller/set_rgb", RGBsState, queue_size=1)
        # rospy.sleep(0.5)  # Delay for publisher setup

        # # Play songs in sequence
        # songs_to_play = [fur_elise, jingle_bells]
        # play_songs_in_sequence(buzzer_pub, rgb_pub, songs_to_play)

        # Keep the main thread alive
        # rospy.spin()

    except:
        print("Program interrupted before completion.")
        # mp3.pause()
        sys.exit(0)
