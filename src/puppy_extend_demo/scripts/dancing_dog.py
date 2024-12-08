#!/usr/bin/python3
#coding=utf8
# Control buzzer to play a simple Christmas tune

import sys
import threading
import queue
import rospy
from ros_robot_controller.msg import BuzzerState
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
    WHOLE = 1.0       # Whole note
    HALF = 0.5        # Half note
    QUARTER = 0.25    # Quarter note
    EIGHTH = 0.125    # Eighth note
    SIXTEENTH = 0.0625 # Sixteenth note


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

    def play_next_note(self, buzzer_pub):
        """Play the next note in the queue."""
        if not self.note_queue.empty():
            note, note_duration = self.note_queue.get()
            buzzer_msg = BuzzerState()
            buzzer_msg.freq = note.value
            buzzer_msg.on_time = note_duration
            buzzer_msg.off_time = 0.1
            buzzer_msg.repeat = 1

            if note == NoteFreq.REST:
                print(f"Rest for {note_duration:.2f} sec")
            else:
                print(f"Playing note: {note.name} ({note.value} Hz) for {note_duration:.2f} sec")

            buzzer_pub.publish(buzzer_msg)
            threading.Timer(note_duration + buzzer_msg.off_time, self.play_next_note, [buzzer_pub]).start()
        else:
            print(f"'{self.name}' complete!")
            if self.on_complete:
                self.on_complete()

    def play(self, buzzer_pub):
        """Start playing the song asynchronously."""
        print(f"Starting '{self.name}' at {self.bpm} BPM")
        self.queue_notes()
        self.play_next_note(buzzer_pub)

def play_songs_in_sequence(buzzer_pub, songs):
    """Play all songs in sequence and exit when done."""
    def play_next():
        if songs:
            current_song = songs.pop(0)
            current_song.on_complete = play_next
            current_song.play(buzzer_pub)
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
    try:
        # Initialize Node
        rospy.init_node('buzzer_song_player')

        buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
        rospy.sleep(0.5)  # Delay for publisher setup


        # Play songs in sequence
        songs_to_play = [fur_elise, jingle_bells]
        play_songs_in_sequence(buzzer_pub, songs_to_play)

        # Keep the main thread alive
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")
        sys.exit(0)
