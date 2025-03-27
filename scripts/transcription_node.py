#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from misty.srv import *
from misty.msg import *
from misty_reading.msg import *

import asyncio
import sounddevice
from datetime import datetime

from amazon_transcribe.client import TranscribeStreamingClient
from amazon_transcribe.handlers import TranscriptResultStreamHandler
from amazon_transcribe.model import TranscriptEvent


class SpeechEventHandler(TranscriptResultStreamHandler):
    def __init__(self, output_stream, stream, publisher):
        super().__init__(output_stream)
        self.transcript = []
        self.stream = stream
        self._pub = publisher
        # Filename for text file is timestamped
        self.filename = datetime.now().strftime('%d %b, %H:%M')


    async def handle_transcript_event(self, transcript_event: TranscriptEvent):
        # handler for transcriptions
        results = transcript_event.transcript.results

        for result in results:
            for alt in result.alternatives:
                transcript_string = ""
                self.transcript = []
                
                for item in alt.items:
                    if item.confidence:
                        self.transcript.append(item.content.lower())
                        transcript_string = transcript_string + ' ' + item.content

                if transcript_string:
                    # Print transcript to console
                    print('=' * 50)
                    print(transcript_string)
                    # Publish transcript
                    transcript_result = Transcription()
                    transcript_result.word_list = self.transcript
                    self._pub.publish(transcript_result)
                    # Log transcript to txt file
                    f = open(self.filename + '.txt', 'a')
                    f.write('\n' + transcript_string)
                    f.close()


class transcriptionNode():
    def __init__(self):
        rospy.init_node("transcription_node")

        # Setup client
        self.client = TranscribeStreamingClient(region="us-east-1")

        # Initiate publisher
        self._pub = rospy.Publisher('transcript', Transcription, queue_size = 10)

        # Create loop and run transcription
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.basic_transcribe())
        self.loop.close()


    async def mic_stream(self):
        # This function wraps the raw input stream from the microphone forwarding
        # the blocks to an asyncio.Queue.
        loop = asyncio.get_event_loop()
        input_queue = asyncio.Queue()

        def callback(indata, frame_count, time_info, status):
            loop.call_soon_threadsafe(input_queue.put_nowait, (bytes(indata), status))

        # Be sure to use the correct parameters for the audio stream that matches
        # the audio formats described for the source language you'll be using:
        # https://docs.aws.amazon.com/transcribe/latest/dg/streaming.html
        stream = sounddevice.RawInputStream(
            channels=1,
            samplerate=16000,
            callback=callback,
            blocksize=1024 * 2,
            dtype="int16",
            device = "pulse"
        )
        # Initiate the audio stream and asynchronously yield the audio chunks
        # as they become available.
        with stream:
            while not rospy.is_shutdown():
                indata, status = await input_queue.get()
                yield indata, status


    async def write_chunks(self, stream):
        # This connects the raw audio chunks generator coming from the microphone
        # and passes them along to the transcription stream.
        async for chunk, status in self.mic_stream():
            await stream.input_stream.send_audio_event(audio_chunk=chunk)
        await stream.input_stream.end_stream()


    async def basic_transcribe(self):
        # Process transcription
        while not rospy.is_shutdown():
            # Start transcription to generate async stream
            stream = await self.client.start_stream_transcription(
                language_code="en-US",
                media_sample_rate_hz=16000,
                media_encoding="pcm",
            )

            # Instantiate handler and start processing events
            handler = SpeechEventHandler(stream.output_stream, self, self._pub)
            await asyncio.gather(self.write_chunks(stream), handler.handle_events())

if __name__ == '__main__':

    transcript_node = transcriptionNode()
    rospy.spin()
