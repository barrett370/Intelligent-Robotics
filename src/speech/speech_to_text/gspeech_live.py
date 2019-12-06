import time
import sys
import os
import threading
# uses result_end_time currently only avaialble in v1p1beta, will be in v1 soon
from google.cloud import speech_v1p1beta1 as speech
import google
import pyaudio
from six.moves import queue
import difflib
# Audio recording parameters
from speech.speech_to_text.instructions import InstructionParser
from speech.speech_to_text.misc_functions import strip_leading_space

STREAMING_LIMIT = 10000
SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE / 10)  # 100ms

RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[0;33m'
ENDC = '\033[0m'
info = lambda x: print(x)
warn = lambda x: print(YELLOW + x + ENDC)
success = lambda x: print(GREEN + x + ENDC)
error = lambda x: print(RED + x + ENDC)


def get_current_time():
    """Return Current Time in MS."""

    return int(round(time.time() * 1000))


class ResumableMicrophoneStream:
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate, chunk_size, mic_id=None):
        self._rate = rate
        self.chunk_size = chunk_size
        self._num_channels = 1
        self._buff = queue.Queue()
        self.closed = True
        self.start_time = get_current_time()
        self.restart_counter = 0
        self.audio_input = []
        self.last_audio_input = []
        self.result_end_time = 0
        self.is_final_end_time = 0
        self.final_request_end_time = 0
        self.bridging_offset = 0
        self.last_transcript_was_final = False
        self.new_stream = True
        self._audio_interface = pyaudio.PyAudio()
        if mic_id is None:
            self._audio_stream = self._audio_interface.open(
                format=pyaudio.paInt16,
                channels=self._num_channels,
                rate=self._rate,
                input=True,
                # input_device_index=mic_id,
                frames_per_buffer=self.chunk_size,
                # Run the audio stream asynchronously to fill the buffer object.
                # This is necessary so that the input device's buffer doesn't
                # overflow while the calling thread makes network requests, etc.
                stream_callback=self._fill_buffer,
            )
        else:
            self._audio_stream = self._audio_interface.open(
                format=pyaudio.paInt16,
                channels=self._num_channels,
                rate=self._rate,
                input=True,
                input_device_index=mic_id,
                frames_per_buffer=self.chunk_size,
                # Run the audio stream asynchronously to fill the buffer object.
                # This is necessary so that the input device's buffer doesn't
                # overflow while the calling thread makes network requests, etc.
                stream_callback=self._fill_buffer,
            )

    def __enter__(self):

        self.closed = False
        return self

    def __exit__(self, type, value, traceback):

        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, *args, **kwargs):
        """Continuously collect data from the audio stream, into the buffer."""

        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        """Stream Audio from microphone to API and to local buffer"""

        while not self.closed:
            data = []

            if self.new_stream and self.last_audio_input:

                chunk_time = STREAMING_LIMIT / len(self.last_audio_input)

                if chunk_time != 0:

                    if self.bridging_offset < 0:
                        self.bridging_offset = 0

                    if self.bridging_offset > self.final_request_end_time:
                        self.bridging_offset = self.final_request_end_time

                    chunks_from_ms = round((self.final_request_end_time -
                                            self.bridging_offset) / chunk_time)

                    self.bridging_offset = (round((
                                                          len(self.last_audio_input) - chunks_from_ms)
                                                  * chunk_time))

                    for i in range(chunks_from_ms, len(self.last_audio_input)):
                        data.append(self.last_audio_input[i])

                self.new_stream = False

            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            self.audio_input.append(chunk)

            if chunk is None:
                return
            data.append(chunk)
            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)

                    if chunk is None:
                        return
                    data.append(chunk)
                    self.audio_input.append(chunk)

                except queue.Empty:
                    break

            yield b''.join(data)


class Listener:

    def __init__(self):
        self.listening_end = 0
        self.parser = InstructionParser()
        self.continued = False
        info('[INFO] creating mic manager')
        self.mic_manager = ResumableMicrophoneStream(SAMPLE_RATE, CHUNK_SIZE)
        self.__thread = threading.Thread(target=self.main, daemon=True)
        success('[INFO] created mic manager')

    def change_mic_id(self, mic_id: int):
        self.mic_manager.__exit__(1, 2, 3)
        self.mic_manager = ResumableMicrophoneStream(SAMPLE_RATE, CHUNK_SIZE, mic_id=mic_id)
        self.__thread.start()

    def parse_input_stream(self, responses):

        # print("attempting to parse input")
        if responses:
            for response in responses:
                # print(1)
                if not response.results:
                    break

                result = response.results[0]
                if not result.alternatives:
                    continue
                if result.is_final:
                    transcript: str = result.alternatives[0].transcript
                    seq = difflib.SequenceMatcher(a="howard", b=transcript.lower()).ratio()
                    # if seq > sim:
                    #     sim = seq
                    # print(2)
                    warn(f"[INFO] Similarity to wake-word is : {seq}")
                    # if transcript.__contains__("Howard"):
                    if seq > 0.7:
                        # trigger motion and vision to scan for speaker (first face detected)
                        # sys.stdout.write(YELLOW)
                        warn("[INFO] Wake Word Detected\n")
                        os.system("mpg321 ../resources/activation.mp3")
                        self.listening_end = get_current_time() + 1000000
                    elif self.listening_end > get_current_time():
                        transcript = strip_leading_space(transcript)
                        info(f"checking for commands {transcript}")
                        # instructions[transcript.lower()]()
                        if self.parser.parse(transcript.lower(), self.continued):
                            success("completed instruction waiting for wake word")
                            self.listening_end = get_current_time()
                            self.continued = False
                        else:
                            warn("Next instruction will be a response")
                            self.continued = True
        # print("returning")
        return

        # Parse following reponses for commands

    def main(self):
        """start bidirectional streaming from microphone input to speech API"""
        client = speech.SpeechClient()
        config = speech.types.RecognitionConfig(
            encoding=speech.enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=SAMPLE_RATE,
            language_code='en-US',
            max_alternatives=1)
        streaming_config = speech.types.StreamingRecognitionConfig(
            config=config,
            interim_results=False)
        # print(self.mic_manager.chunk_size)
        with self.mic_manager as stream:

            while not stream.closed:

                stream.audio_input = []
                audio_generator = stream.generator()

                requests = (speech.types.StreamingRecognizeRequest(
                    audio_content=content) for content in audio_generator)

                responses = client.streaming_recognize(streaming_config, requests)
                # Now, put the transcription responses to use.
                try:
                    info("Parsing input")
                    self.parse_input_stream(responses)
                except google.api_core.exceptions.DeadlineExceeded:
                    # print("cannot parse")
                    continue
                # listen_print_loop(responses, stream)

                if stream.result_end_time > 0:
                    stream.final_request_end_time = stream.is_final_end_time
                stream.result_end_time = 0
                stream.last_audio_input = []
                stream.last_audio_input = stream.audio_input
                stream.audio_input = []
                stream.restart_counter = stream.restart_counter + 1

                if not stream.last_transcript_was_final:
                    sys.stdout.write('\n')
                # stream.new_stream = True
