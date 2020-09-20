#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import os
import re
import sys
import time
import wave
import struct
import scipy
import rospy
import signal
import pyaudio
import pyttsx3
import datetime
import numpy as np
from six.moves import queue
from datetime import datetime
from std_msgs.msg import String, Bool
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.api_core import exceptions

# Note these changes I made after running "pip install noisereduce --user"
# https://github.com/timsainb/noisereduce/issues/23

# Note that Google Speech-to-Text does not want you to apply noise-reduction
# on the input signal:
# https://cloud.google.com/speech-to-text/docs/best-practices
# We only apply it on the signal we save off for our own internal purposes

#########################################################
# setup a signal handler to listen for exit signal (ctrl + c)
#########################################################
def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

#########################################################
# get current time
#########################################################
def get_current_time():
	return int(round(time.time() * 1000))

#########################################################
# define microphone stream object
#########################################################
class MicrophoneStream(object):
	"""Opens a recording stream as a generator yielding the audio chunks."""
	def __init__(self, rate, chunk_size, client_id):
		self._rate = rate
		self._chunk_size = chunk_size
		self._num_channels = 1
		self._max_replay_secs = 5
		self.client_id = client_id

		# Create a thread-safe buffer of audio data
		self._buff = queue.Queue()
		self.closed = True
		self.start_time = get_current_time()

		# 2 bytes in 16 bit samples
		self._bytes_per_sample = 2 * self._num_channels
		self._bytes_per_second = self._rate * self._bytes_per_sample

		self._bytes_per_chunk = (self._chunk_size * self._bytes_per_sample)
		self._chunks_per_second = (self._bytes_per_second // self._bytes_per_chunk)

	def __enter__(self):
		self.closed = False
		self._audio_interface = pyaudio.PyAudio()
		self._audio_stream = self._audio_interface.open(
			format=pyaudio.paInt16,
			channels=self._num_channels,
			rate=self._rate,
			input_device_index=args['microphone_id'],
			input=True,
			frames_per_buffer=self._chunk_size,
			# Run the audio stream asynchronously to fill the buffer object.
			# This is necessary so that the input device's buffer doesn't
			# overflow while the calling thread makes network requests, etc.
			stream_callback=self._fill_buffer,
		)
		return self

	def __exit__(self, type, value, traceback):
		self._audio_stream.stop_stream()
		self._audio_stream.close()
		self.closed = True
		# Signal the generator to terminate so that the client's
		# streaming_recognize method will not block the process termination.
		self._buff.put(None)
		self._audio_interface.terminate()

	def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
		"""Continuously collect data from the audio stream, into the buffer."""
		global run_speech_to_text, record, record_frames, silent_start_timer, indicated_silence

		# if the robot asked a question, check that we aren't letting silence last forever
		if not silent_start_timer == None:
			if run_speech_to_text:
				silent_time_elapsed =  time.time() - silent_start_timer
				#print 'silent_time_elapsed:', silent_time_elapsed
				if run_speech_to_text and silent_time_elapsed > args['silence_timeout'] and not indicated_silence:
						human_speaking_pub.publish(False)
						ears_pub.publish('<silence>')
						log_message("Heard: <silence>")
						indicated_silence = True

		if run_speech_to_text or record:
			self._buff.put(in_data)
			if record: record_frames.append(in_data)
		if not run_speech_to_text:
			self._buff.put(chr(0)*frame_count*16)
		return None, pyaudio.paContinue

	def generator(self):
		global current_client_id
		t0 = time.time()
		last_sec_elapsed = 0
		while not self.closed:
			time_since_start = time.time() - t0
			sec_elapsed = int(str(time_since_start).split('.')[0])
			if not sec_elapsed == last_sec_elapsed:
				log_message('sec_elapsed: ' + str(sec_elapsed) + '. listening on ' + self.client_id)
				last_sec_elapsed = sec_elapsed
			if not self.client_id == current_client_id:
				log_message('access_point_changed: ' + self.client_id + ' ' + current_client_id)
				break
			if get_current_time() - self.start_time > args['streaming_limit']:
				self.start_time = get_current_time()
				break
			# Use a blocking get() to ensure there's at least one chunk of
			# data, and stop iteration if the chunk is None, indicating the
			# end of the audio stream.
			chunk = self._buff.get()
			if chunk is None:
				return
			data = [chunk]

			# Now consume whatever other data's still buffered.
			while True:
				try:
					chunk = self._buff.get(block=False)
					if chunk is None:
						return
					data.append(chunk)
				except queue.Empty:
					break

			yield b''.join(data)

#########################################################
# publish speech-to-text results
# Iterates through server responses and prints them.
#
# The responses passed is a generator that will block until a response
# is provided by the server.
# 
# Each response may contain multiple results, and each result may contain
# multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
# print only the transcription for the top alternative of the top result.
#
# In this case, responses are provided for interim results as well. If the
# response is an interim one, print a line feed at the end of it, to allow
# the next result to overwrite it, until the response is a final one. For the
# final one, print a newline to preserve the finalized transcription.
#########################################################
def listen_print_loop(responses):
	global ears_pub, human_speaking_pub, silent_start_timer

	num_chars_printed = 0
	
	for response in responses:
		if not response.results:
			continue

		# once the robot has asked an initial question, every time we hear a result, restart our silent timer
		if not silent_start_timer == None:
			silent_start_timer = time.time()
			indicated_silence  = False

		# The `results` list is consecutive. For streaming, we only care about
		# the first result being considered, since once it's `is_final`, it
		# moves on to considering the next utterance.
		result = response.results[0]
		if not result.alternatives:
			continue

		# Display the transcription of the top alternative.
		transcript = result.alternatives[0].transcript

		# Display interim results, but with a carriage return at the end of the
		# line, so subsequent lines will overwrite them.
		#
		# If the previous result was longer than this one, we need to print
		# some extra spaces to overwrite the previous result
		overwrite_chars = ' ' * (num_chars_printed - len(transcript))

		if not result.is_final:
			human_speaking_pub.publish(True)
			sys.stdout.write(transcript + overwrite_chars + '\r')
			sys.stdout.flush()
			num_chars_printed = len(transcript)
		else:
			human_speaking_pub.publish(False)
			ears_pub.publish(transcript + overwrite_chars)
			num_chars_printed = 0
			log_message("Heard: " + transcript + overwrite_chars)
			#raise exceptions.OutOfRange('Jared did this to test something.')

#########################################################
# microphone streaming loop
#########################################################
def microphone_streaming_loop(this_client_id):
	global listen, silent_start_timer, indicated_silence, current_mic_streams, current_client_id
	log_message('Init mouth_and_ears ' + this_client_id)

	while this_client_id == current_client_id:
		reconnect_time = 0.0
		# if we are told to listen, then repeatedly try to listen (in the event something crashes)!
		while listen and this_client_id == current_client_id:
			log_message('Attempt to start mouth_and_ears {0} in {1:0.1f} seconds'.format(this_client_id, reconnect_time))
			time.sleep(reconnect_time)
			try:
				t1 = time.time()
				# set microphone_id to the one specified by the user
				args['microphone_id'] = 0
				p = pyaudio.PyAudio()
				info = p.get_host_api_info_by_index(0)
				numdevices = info.get('deviceCount')
				log_message('Available microphones:')
				for i in range(0, numdevices):
					if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
						microphone_name = p.get_device_info_by_host_api_device_index(0, i).get('name')
						log_message("\tInput Device id " + str(i) + " - " + microphone_name)
						if args['microphone'].lower() in microphone_name.lower():
							args['microphone_id'] = i
				log_message('Using microphone' + str(args['microphone_id']))

				# setup microphone with speech-to-text client
				client = speech.SpeechClient()
				# SpeechContext: to configure your speech_context see:
				# https://cloud.google.com/speech-to-text/docs/reference/rpc/google.cloud.speech.v1#speechcontext
				# Full list of supported phrases (class tokens) here:
				# https://cloud.google.com/speech-to-text/docs/class-tokens
				common_phrases=['$OOV_CLASS_DIGIT_SEQUENCE', '$OPERAND', 'hosh', 'hallway', 'three-way', \
							    'elbow', 'corner', 'end', 'go', 'take', 'turn', 'around', 'hall', \
							    'room', 'window', 'door', 'down', 'ahead', 'left', 'right', 'forward', 'straight', \
							    'find', 'jared', 'tom', 'jeff']
				speech_context = speech.types.SpeechContext(phrases=common_phrases)
				config = types.RecognitionConfig(encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
												 sample_rate_hertz=args['rate'], language_code='en-US', 
												 enable_automatic_punctuation=True,
												 speech_contexts=[speech_context])
				streaming_config = types.StreamingRecognitionConfig(config=config, interim_results=True, single_utterance=False)
				mic_manager = MicrophoneStream(args['rate'], args['chunk'], current_client_id)

				# listen forever to the audio stream, shipping it to google for speech-to-text, then publishing it.
				log_message("Listening!")
				t2 = time.time()
				log_message("Time to get setup to start listening: " + str(t2-t1))
				with mic_manager as stream:
					while not stream.closed:
						audio_generator = stream.generator()
						requests  = (types.StreamingRecognizeRequest(audio_content=content) for content in audio_generator)
						responses = client.streaming_recognize(streaming_config, requests)
						current_mic_streams.append(mic_manager)
						log_message('looping ' + this_client_id)
						# Now, put the transcription responses to use.
						listen_print_loop(responses)
						if not listen:
							silent_start_timer = None
							indicated_silence  = False
							break
						if not this_client_id == current_client_id:
							raise Exception('Access Point Changed.  Killing ' + this_client_id + ' since client == ' + current_client_id)
			except Exception as e:
				log_message('caught error: %s'%e)
				reconnect_time += 0.1
		if not listen:
			time.sleep(0.1)
	log_message('Done mouth_and_ears ' + this_client_id)

def listenerA(data):
	log_message('listenerA')
	microphone_streaming_loop('A')
def listenerB(data):
	log_message('listenerB')
	microphone_streaming_loop('B')
def listenerC(data):
	log_message('listenerC')
	microphone_streaming_loop('C')
def listenerD(data):
	log_message('listenerD')
	microphone_streaming_loop('D')
def listenerE(data):
	log_message('listenerE')
	microphone_streaming_loop('E')

#########################################################
# main loop
#########################################################
def main_loop():
	global last_network_time, last_access_point_address, network_log_file, current_client_id, current_mic_streams

	# start listening
	listenerA_pub.publish(True)
	time.sleep(5)

	while True:
		# check to see if we connected to a new access point
		cur_net_file_time = os.stat(network_log_file).st_mtime
		if not cur_net_file_time == last_network_time:
			log_message('network log file update!' + str(cur_net_file_time) + ' ' + str(last_network_time))
			last_network_time = cur_net_file_time
			f = open(network_log_file)
			while True:
				line = f.readline()
				if not line: break
				access_point_address = last_access_point_address
				if 'New Access Point/Cell address' in line:
					access_point_address = line.strip().split('address:')[1]
			if not access_point_address == 'Not-Associated' and not access_point_address == last_access_point_address:
				last_access_point_address = access_point_address
				log_message('last_access_point_address: ' + last_access_point_address)
				for idx, current_mic_stream in enumerate(current_mic_streams):
					if not current_mic_stream == None:
						try:
							log_message('stopping microphone stream ' + current_mic_stream.client_id)
							current_mic_stream._audio_stream.stop_stream()
							current_mic_stream._audio_stream.close()
							current_mic_stream.closed = True
							current_mic_stream._buff.put(None)
							current_mic_stream._audio_interface.terminate()
							current_mic_streams[idx] = None
							log_message('stopping current microphone stream ' + current_mic_stream.client_id + ': done')
						except:
							log_message('Failure closing stream: ' + current_mic_stream.client_id)

				if current_client_id == 'A': 
					current_client_id = 'B'
					listenerB_pub.publish(True)
				elif current_client_id == 'B': 
					current_client_id = 'C'
					listenerC_pub.publish(True)
				elif current_client_id == 'C': 
					current_client_id = 'D'
					listenerD_pub.publish(True)
				elif current_client_id == 'D': 
					current_client_id = 'E'
					listenerE_pub.publish(True)
				elif current_client_id == 'E': 
					current_client_id = 'A'
					listenerA_pub.publish(True)
		else:
			time.sleep(0.1)

#########################################################
# returns the contents of the wav file as a double precision float array
#########################################################
def wav_to_floats(wave_file):
	w = wave.open(wave_file)
	astr = w.readframes(w.getnframes())
	# convert binary chunks to short
	a = struct.unpack("%ih" % (w.getnframes()* w.getnchannels()), astr)
	a = [float(val) / pow(2, 15) for val in a]
	return np.array(a)

######################################################################################
# save off audio clip
######################################################################################
''''
def save_audio():
	# refer to global objects
	global args, record_frames, audio_filename

	# convert binary chunks to short 
	audio_data = struct.unpack("%ih" % (len(record_frames)*mic_manager._num_channels*args['chunk']), b''.join(record_frames))
	audio_data = [float(val) / pow(2, 15) for val in audio_data]

	# perform noise reduction
	reduced_noise = nr.reduce_noise(audio_clip=audio_data, noise_clip=noise_data, verbose=False)

	# save off audio stream
	scipy.io.wavfile.write(audio_filename, args['rate'], record_frames)

	# reset our record frames
	record_frames = []
'''
######################################################################################
# save off audio clip
######################################################################################
def save_audio():
	# refer to global objects
	global args, record_frames, audio_filename

	# save off audio stream (if user requested such)
	waveFile = wave.open(audio_filename, 'wb')
	waveFile.setnchannels(1)
	waveFile.setsampwidth(pyaudio.PyAudio().get_sample_size(pyaudio.paInt16))
	waveFile.setframerate(args['rate'])
	waveFile.writeframes(b''.join(record_frames))

	# reset our record frames
	record_frames = []

######################################################################################
# add an utterance to the list of things the robot needs to say
######################################################################################
def add_thing_to_say(data):
	# refer to global objects
	global things_to_say
	
	# add thing to say
	if not data.data in things_to_say:
		things_to_say.append(data.data)

######################################################################################
# add an utterance to the list of things the robot needs to say
######################################################################################
def clear_thing_to_say(data):
	# refer to global objects
	global things_to_say
	
	# remove things to say
	things_to_say = []

######################################################################################
# synthesize speech
######################################################################################
def synthesize_speech(data):
	# refer to global objects
	global run_speech_to_text, speech_engine, things_to_say, silent_start_timer, indicated_silence

	while data.data:
		if len(things_to_say) > 0:
			# stop running speech-to-text
			run_speech_to_text = False

			# synthesize speech
			robot_speaking_pub.publish(True)
			thing_to_say = things_to_say[0]
			log_message("Speak: " + thing_to_say)
			speech_engine.say(thing_to_say)
			speech_engine.runAndWait()
			if len(things_to_say) > 0 and things_to_say[0] == thing_to_say:
				things_to_say.pop(0)
			if len(things_to_say) == 0:
				robot_speaking_pub.publish(False)

			# restart our silent timer
			silent_start_timer = time.time()
			indicated_silence  = False

			# start running speech-to-text again
			run_speech_to_text = True

######################################################################################
# record_mouth_and_ears
######################################################################################
def record_mouth_and_ears(data):
	# refer to global objects
	global record, conv_path, audio_filename, listen

	# set record flag 
	record = data.data

	# if told to start recording, save the timestamp
	if record:
		audio_filename = conv_path + '/conv-' + datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + '.wav'
		listen = True

	# if told to stop recording, save the recording
	if record == False:
		save_audio()
		listen = False
		
######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	timestamp = datetime.now().strftime("%Y-%b-%d-%H-%M-%S")
	print timestamp, msg
	args['output_log'].write(timestamp + ' ' + msg + '\n')

######################################################################################
# when starting up...
######################################################################################
if __name__ == '__main__':
	# import launch file params
	args = {}
	args['microphone']           = rospy.get_param('mouth_and_ears/microphone')
	args['rate']                 = rospy.get_param('mouth_and_ears/rate')
	args['streaming_limit']      = rospy.get_param('mouth_and_ears/streaming_limit')
	args['chunk']                = rospy.get_param('mouth_and_ears/chunk')
	args['silence_timeout']      = rospy.get_param('mouth_and_ears/silence_timeout')

	# open output log
	output_filename    = datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/mouth_and_ears/log/'
	conv_path          = home + '/.ros/hold_conversation/conv/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	if not os.path.exists(conv_path):
		os.makedirs(conv_path)
	args['output_log'] = open(log_path + output_filename, 'w')

	# initialize some global variables
	record                    = False
	listen                    = True
	run_speech_to_text        = True
	record_frames             = []
	things_to_say             = []
	audio_filename            = conv_path + '/init.wav'
	silent_start_timer        = None
	indicated_silence         = False
	network_log_file          = home + '/robot-slang/scripts/iwevent_log.txt'
	last_network_time         = os.stat(network_log_file).st_mtime
	last_access_point_address = 'None'
	current_mic_streams       = []
	current_client_id         = 'A'

	# load noise data
	noise_file = home + '/robot-slang/husky-custom/src/mouth_and_ears/src/noise.wav'
	noise_data = wav_to_floats(noise_file)

	# intialize node and setup subscriptions/publications
	try:
		# name our node
		rospy.init_node('mouth_and_ears')
		log_message('Starting mouth and ears')

		# setup subscribers
		rospy.Subscriber('/mouth_and_ears/clear_say_buffer', Bool, clear_thing_to_say)
		rospy.Subscriber('/mouth_and_ears/say', String, add_thing_to_say)
		rospy.Subscriber('/mouth_and_ears/start_talking', Bool, synthesize_speech)
		rospy.Subscriber('/mouth_and_ears/record', Bool, record_mouth_and_ears)
		rospy.Subscriber('/mouth_and_ears/listenerA', Bool, listenerA)
		rospy.Subscriber('/mouth_and_ears/listenerB', Bool, listenerB)
		rospy.Subscriber('/mouth_and_ears/listenerC', Bool, listenerC)
		rospy.Subscriber('/mouth_and_ears/listenerD', Bool, listenerD)
		rospy.Subscriber('/mouth_and_ears/listenerE', Bool, listenerE)

		# setup publishers
		ears_pub           = rospy.Publisher('/mouth_and_ears/heard_speech', String, queue_size=1)
		human_speaking_pub = rospy.Publisher('/mouth_and_ears/human_speaking', Bool, queue_size=1)
		robot_speaking_pub = rospy.Publisher('/mouth_and_ears/robot_speaking', Bool, queue_size=1)
		start_talking_pub  = rospy.Publisher('/mouth_and_ears/start_talking',  Bool, queue_size=1)
		listenerA_pub  = rospy.Publisher('/mouth_and_ears/listenerA',  Bool, queue_size=1)
		listenerB_pub  = rospy.Publisher('/mouth_and_ears/listenerB',  Bool, queue_size=1)
		listenerC_pub  = rospy.Publisher('/mouth_and_ears/listenerC',  Bool, queue_size=1)
		listenerD_pub  = rospy.Publisher('/mouth_and_ears/listenerD',  Bool, queue_size=1)
		listenerE_pub  = rospy.Publisher('/mouth_and_ears/listenerE',  Bool, queue_size=1)

		# wait one second, then set flag to allow robot to start talking
		time.sleep(1)
		start_talking_pub.publish(True)

		# setup speech engine
		# Random note: if the speech_engine is within the try/except block below, and the except statement fires,
		# and everything gets restarted, the speech_engine is have the rate it was before.  I moved it out here to 
		# prevent that.  (It's not really needed in the try/except anyway since it is just the ears that timeout.
		speech_engine = pyttsx3.init()
		rate = speech_engine.getProperty('rate')
		speech_engine.setProperty('rate', rate-50)
		speech_engine.setProperty('voice','english-us') # english-us, en-scottish, en-scottish, english
		speech_engine.setProperty('gender', 'female')
		log_message("Ready to speak!")

		# go into main loop
		main_loop()
	except rospy.ROSInterruptException: 
		pass

# export GOOGLE_APPLICATION_CREDENTIALS=/home/jjohanse/robot-slang/husky-custom/src/dialog/src/Dialog.json
# source /home/jjohanse/robot-slang/husky-custom/devel/setup.bash 
# rostopic pub -1 /mouth_and_ears/start_talking std_msgs/Bool 1
# rostopic pub -1 /mouth_and_ears/say std_msgs/String "hello"

