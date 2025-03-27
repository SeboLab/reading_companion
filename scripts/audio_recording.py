#!/usr/bin/env python
import pyaudio
import wave
import rospy
from std_msgs.msg import String, Empty, Bool
import os
import time

filename = "temp.wav"
failsafe = False

def start_audio_recording():
    global filename, failsafe
    chunk = 1024  # Record in chunks of 1024 samples
    sample_format = pyaudio.paInt16  # 16 bits per sample
    channels = 2
    fs = 44100  # Record at 44100 samples per second
    audio_recordings_dir = os.path.expanduser("~") + '/catkin_ws/src/misty_reading/scripts/recordings/'
    # Verify recordings directory exists, if not create it.
    if not os.path.exists(audio_recordings_dir):
        os.makedirs(audio_recordings_dir)

    p = pyaudio.PyAudio()

    stream = p.open(format=sample_format,
                    channels=channels,
                    rate=fs,
                    frames_per_buffer=chunk,
                    input=True)

    frames = []  # Initialize array to store frames

    while not failsafe:
        data = stream.read(chunk)
        frames.append(data)

    time.sleep(1)
    # Stop and close the stream 
    stream.stop_stream()
    stream.close()
    # Terminate the PortAudio interface
    p.terminate()

    # Save the recorded data as a WAV file

    if "_" in filename:
        profile_id = filename.split('_')[0]
        audio_recordings_dir = audio_recordings_dir.replace("recordings/", profile_id + "/recordings/")
        # Verify recordings directory exists, if not create it.
        if not os.path.exists(audio_recordings_dir):
            os.makedirs(audio_recordings_dir)
    filepath = audio_recordings_dir + filename
    wf = wave.open(filepath, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(sample_format))
    wf.setframerate(fs)
    wf.writeframes(b''.join(frames))
    wf.close()

def endRecording(data):
    global failsafe
    failsafe = True
        
def updateFilename(data):
    global filename
    filename = data.data

if __name__ == "__main__":
    rospy.init_node('audio_recording')
    eR = rospy.Subscriber('misty/stopAudioRecording', Bool, endRecording)
    rF = rospy.Subscriber('misty/getRecordingFilename', String, updateFilename)
    sD = rospy.Subscriber('misty/command/killTopics', Empty, endRecording)
    start_audio_recording()