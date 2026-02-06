---
sidebar_position: 4
title: "Chapter 24: Voice-to-Action with Whisper"
description: "Enable humanoid robots to understand and respond to voice commands"
---

# Chapter 24: Voice-to-Action with Whisper

Voice is the most natural interface for human-robot interaction. This chapter covers integrating OpenAI's Whisper speech recognition with ROS 2 to enable humanoid robots to understand and execute voice commands.

## Learning Objectives

By the end of this chapter, you will be able to:

- Integrate Whisper for speech-to-text on robots
- Design voice command parsing systems
- Map natural language to robot actions
- Handle continuous voice interaction
- Implement wake word detection

## 24.1 Speech Recognition for Robotics

### Challenges in Robot Environments

| Challenge | Description | Solution |
|-----------|-------------|----------|
| Noise | Motor noise, environment | Noise-robust models |
| Distance | Far-field microphones | Beamforming |
| Real-time | Low latency needed | Streaming ASR |
| Vocabulary | Domain-specific terms | Fine-tuning |

### Whisper Overview

**Whisper** is OpenAI's speech recognition model:

- Trained on 680,000 hours of multilingual data
- Supports 99 languages
- Robust to noise and accents
- Available in multiple sizes (tiny to large)

```
Audio Input → Whisper → Text → Command Parser → Robot Action
```

## 24.2 Whisper Integration

### Installation

```bash
# Install Whisper
pip install openai-whisper

# Or faster-whisper for optimized inference
pip install faster-whisper

# Audio dependencies
sudo apt install portaudio19-dev
pip install pyaudio sounddevice
```

### Basic Usage

```python
import whisper

# Load model
model = whisper.load_model("base")  # tiny, base, small, medium, large

# Transcribe audio file
result = model.transcribe("command.wav")
print(result["text"])

# With language specification
result = model.transcribe("command.wav", language="en")
```

### Streaming Recognition

```python
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel

class StreamingWhisper:
    def __init__(self, model_size="base"):
        self.model = WhisperModel(model_size, device="cuda", compute_type="float16")
        self.sample_rate = 16000
        self.chunk_duration = 3.0  # seconds
        self.buffer = []
    
    def start_listening(self, callback):
        """Start continuous listening with callback for transcriptions."""
        self.callback = callback
        self.running = True
        
        def audio_callback(indata, frames, time, status):
            if status:
                print(f"Audio status: {status}")
            self.buffer.extend(indata[:, 0])
            
            # Process when we have enough audio
            if len(self.buffer) >= self.sample_rate * self.chunk_duration:
                self.process_chunk()
        
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=audio_callback
        ):
            while self.running:
                sd.sleep(100)
    
    def process_chunk(self):
        """Process accumulated audio."""
        audio = np.array(self.buffer[:int(self.sample_rate * self.chunk_duration)])
        self.buffer = self.buffer[int(self.sample_rate * self.chunk_duration):]
        
        # Transcribe
        segments, info = self.model.transcribe(audio, beam_size=5)
        
        for segment in segments:
            if segment.no_speech_prob < 0.5:  # Filter silence
                self.callback(segment.text)
    
    def stop(self):
        self.running = False
```

## 24.3 ROS 2 Voice Node

### Voice Command Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from faster_whisper import WhisperModel
import numpy as np
import sounddevice as sd
from threading import Thread
import queue

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration', 2.0)
        
        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        
        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = WhisperModel(model_size, device="cuda")
        
        # Publishers
        self.transcription_pub = self.create_publisher(String, '/voice/transcription', 10)
        self.command_pub = self.create_publisher(String, '/voice/command', 10)
        
        # Audio buffer
        self.audio_queue = queue.Queue()
        
        # Start audio capture thread
        self.running = True
        self.audio_thread = Thread(target=self.capture_audio)
        self.audio_thread.start()
        
        # Processing timer
        self.timer = self.create_timer(self.chunk_duration, self.process_audio)
        
        self.get_logger().info('Voice command node ready')
    
    def capture_audio(self):
        """Capture audio in background thread."""
        def callback(indata, frames, time, status):
            if status:
                self.get_logger().warn(f'Audio status: {status}')
            self.audio_queue.put(indata.copy())
        
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=callback
        ):
            while self.running:
                sd.sleep(100)
    
    def process_audio(self):
        """Process accumulated audio chunks."""
        # Collect audio from queue
        audio_chunks = []
        while not self.audio_queue.empty():
            audio_chunks.append(self.audio_queue.get())
        
        if not audio_chunks:
            return
        
        audio = np.vstack(audio_chunks).flatten()
        
        # Skip if too quiet (likely silence)
        if np.abs(audio).max() < 0.01:
            return
        
        # Transcribe
        segments, info = self.model.transcribe(
            audio,
            language=self.language,
            beam_size=5,
            vad_filter=True  # Voice activity detection
        )
        
        for segment in segments:
            text = segment.text.strip()
            if text and segment.no_speech_prob < 0.5:
                self.get_logger().info(f'Transcribed: {text}')
                
                # Publish transcription
                msg = String()
                msg.data = text
                self.transcription_pub.publish(msg)
                
                # Parse and publish command
                command = self.parse_command(text)
                if command:
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.command_pub.publish(cmd_msg)
    
    def parse_command(self, text):
        """Extract robot command from transcription."""
        text_lower = text.lower()
        
        # Simple keyword matching
        if any(word in text_lower for word in ['stop', 'halt', 'freeze']):
            return 'STOP'
        elif 'forward' in text_lower or 'go ahead' in text_lower:
            return 'MOVE_FORWARD'
        elif 'backward' in text_lower or 'go back' in text_lower:
            return 'MOVE_BACKWARD'
        elif 'left' in text_lower:
            return 'TURN_LEFT'
        elif 'right' in text_lower:
            return 'TURN_RIGHT'
        elif 'pick up' in text_lower or 'grab' in text_lower:
            return 'PICK_UP'
        elif 'put down' in text_lower or 'release' in text_lower:
            return 'PUT_DOWN'
        elif 'wave' in text_lower:
            return 'WAVE'
        elif 'hello' in text_lower or 'hi' in text_lower:
            return 'GREET'
        
        return None
    
    def destroy_node(self):
        self.running = False
        self.audio_thread.join()
        super().destroy_node()

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 24.4 Wake Word Detection

### Implementing Wake Word

```python
import pvporcupine
import struct

class WakeWordDetector:
    def __init__(self, wake_word="jarvis"):
        # Porcupine wake word detection
        self.porcupine = pvporcupine.create(
            access_key="YOUR_ACCESS_KEY",
            keywords=[wake_word]
        )
        self.frame_length = self.porcupine.frame_length
        self.sample_rate = self.porcupine.sample_rate
    
    def process_frame(self, audio_frame):
        """Process audio frame, return True if wake word detected."""
        pcm = struct.unpack_from("h" * self.frame_length, audio_frame)
        keyword_index = self.porcupine.process(pcm)
        return keyword_index >= 0
    
    def cleanup(self):
        self.porcupine.delete()

class VoiceWithWakeWord(Node):
    def __init__(self):
        super().__init__('voice_wake_word')
        
        self.wake_detector = WakeWordDetector("hey robot")
        self.whisper = WhisperModel("base")
        
        self.listening_for_command = False
        self.command_timeout = 5.0  # seconds
        
    def audio_callback(self, audio_chunk):
        if not self.listening_for_command:
            # Check for wake word
            if self.wake_detector.process_frame(audio_chunk):
                self.get_logger().info("Wake word detected!")
                self.listening_for_command = True
                self.start_command_timer()
                self.play_acknowledgment()
        else:
            # Process as command
            self.command_buffer.extend(audio_chunk)
    
    def start_command_timer(self):
        """Start timeout for command listening."""
        self.command_timer = self.create_timer(
            self.command_timeout,
            self.command_timeout_callback
        )
    
    def command_timeout_callback(self):
        """Called when command listening times out."""
        if self.command_buffer:
            self.process_command()
        self.listening_for_command = False
        self.command_timer.cancel()
    
    def play_acknowledgment(self):
        """Play sound to indicate robot is listening."""
        # Play a beep or say "Yes?"
        pass
```

## 24.5 Natural Language Command Parsing

### Intent Classification

```python
from transformers import pipeline

class IntentClassifier:
    def __init__(self):
        # Zero-shot classification
        self.classifier = pipeline(
            "zero-shot-classification",
            model="facebook/bart-large-mnli"
        )
        
        self.intents = [
            "navigation",
            "manipulation",
            "greeting",
            "question",
            "stop",
            "status_query"
        ]
    
    def classify(self, text):
        """Classify intent of voice command."""
        result = self.classifier(text, self.intents)
        
        return {
            'intent': result['labels'][0],
            'confidence': result['scores'][0]
        }

class SemanticCommandParser:
    def __init__(self):
        self.intent_classifier = IntentClassifier()
        
        # Entity extraction patterns
        self.location_patterns = [
            r'to the (\w+)',
            r'to (\w+)',
            r'at the (\w+)',
            r'near the (\w+)'
        ]
        
        self.object_patterns = [
            r'pick up the (\w+)',
            r'grab the (\w+)',
            r'get the (\w+)',
            r'put down the (\w+)',
            r'place the (\w+)'
        ]
    
    def parse(self, text):
        """Parse voice command into structured format."""
        import re
        
        # Get intent
        intent_result = self.intent_classifier.classify(text)
        
        # Extract entities
        entities = {}
        
        # Extract locations
        for pattern in self.location_patterns:
            match = re.search(pattern, text.lower())
            if match:
                entities['location'] = match.group(1)
                break
        
        # Extract objects
        for pattern in self.object_patterns:
            match = re.search(pattern, text.lower())
            if match:
                entities['object'] = match.group(1)
                break
        
        # Extract numbers (for distances, quantities)
        numbers = re.findall(r'\b(\d+(?:\.\d+)?)\s*(meters?|m|feet|steps?)?', text.lower())
        if numbers:
            value, unit = numbers[0]
            entities['distance'] = {
                'value': float(value),
                'unit': unit or 'meters'
            }
        
        return {
            'intent': intent_result['intent'],
            'confidence': intent_result['confidence'],
            'entities': entities,
            'original_text': text
        }
```

## 24.6 Command Execution

### Voice Command Executor

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

class VoiceCommandExecutor(Node):
    def __init__(self):
        super().__init__('voice_executor')
        
        # Command parser
        self.parser = SemanticCommandParser()
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/voice/transcription',
            self.command_callback, 10
        )
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Service clients
        self.wave_client = self.create_client(Trigger, '/wave')
        self.pick_client = self.create_client(Trigger, '/pick_object')
        
        # Known locations
        self.locations = {
            'kitchen': [2.0, 1.0, 0.0],
            'living room': [0.0, 3.0, 0.0],
            'door': [-1.0, 0.0, 1.57],
            'table': [1.5, 2.0, 0.0],
        }
        
        self.get_logger().info('Voice command executor ready')
    
    def command_callback(self, msg):
        """Process incoming voice commands."""
        text = msg.data
        
        # Parse command
        parsed = self.parser.parse(text)
        self.get_logger().info(f"Parsed: {parsed}")
        
        if parsed['confidence'] < 0.5:
            self.say("I didn't understand that")
            return
        
        # Execute based on intent
        intent = parsed['intent']
        entities = parsed['entities']
        
        if intent == 'navigation':
            self.handle_navigation(entities)
        elif intent == 'manipulation':
            self.handle_manipulation(entities)
        elif intent == 'greeting':
            self.handle_greeting()
        elif intent == 'stop':
            self.handle_stop()
        elif intent == 'status_query':
            self.handle_status_query(text)
    
    def handle_navigation(self, entities):
        """Handle navigation commands."""
        if 'location' in entities:
            location_name = entities['location']
            
            if location_name in self.locations:
                pose = self.locations[location_name]
                self.navigate_to(pose)
                self.say(f"Going to {location_name}")
            else:
                self.say(f"I don't know where {location_name} is")
        
        elif 'distance' in entities:
            distance = entities['distance']['value']
            self.move_forward(distance)
            self.say(f"Moving {distance} meters")
        
        else:
            # Simple movement
            self.move_forward(1.0)
    
    def handle_manipulation(self, entities):
        """Handle manipulation commands."""
        if 'object' in entities:
            object_name = entities['object']
            self.say(f"Looking for {object_name}")
            
            # Call pick service
            request = Trigger.Request()
            future = self.pick_client.call_async(request)
            future.add_done_callback(self.pick_callback)
    
    def handle_greeting(self):
        """Handle greeting commands."""
        self.say("Hello! How can I help you?")
        
        # Wave gesture
        request = Trigger.Request()
        self.wave_client.call_async(request)
    
    def handle_stop(self):
        """Stop all motion."""
        twist = Twist()  # Zero velocity
        self.vel_pub.publish(twist)
        self.say("Stopping")
    
    def navigate_to(self, pose):
        """Send navigation goal."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose[0]
        goal.pose.position.y = pose[1]
        goal.pose.orientation.z = np.sin(pose[2] / 2)
        goal.pose.orientation.w = np.cos(pose[2] / 2)
        
        self.goal_pub.publish(goal)
    
    def move_forward(self, distance):
        """Move forward specified distance."""
        twist = Twist()
        twist.linear.x = 0.3  # m/s
        
        # Calculate duration
        duration = distance / 0.3
        
        # Publish for duration
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop
        self.vel_pub.publish(Twist())
    
    def say(self, text):
        """Text-to-speech output."""
        # Implement TTS (covered in next section)
        self.get_logger().info(f"Robot says: {text}")
```

## 24.7 Text-to-Speech Response

### TTS Integration

```python
import pyttsx3
from gtts import gTTS
import os

class TextToSpeech:
    def __init__(self, method='pyttsx3'):
        self.method = method
        
        if method == 'pyttsx3':
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)
            self.engine.setProperty('volume', 0.9)
        
    def speak(self, text):
        """Convert text to speech."""
        if self.method == 'pyttsx3':
            self.engine.say(text)
            self.engine.runAndWait()
        
        elif self.method == 'gtts':
            # Google TTS (requires internet)
            tts = gTTS(text=text, lang='en')
            tts.save('/tmp/speech.mp3')
            os.system('mpg123 -q /tmp/speech.mp3')
    
    def speak_async(self, text):
        """Non-blocking speech."""
        from threading import Thread
        thread = Thread(target=self.speak, args=(text,))
        thread.start()
```

## 24.8 Complete Voice Interface

### Full Integration

```python
class VoiceInterface(Node):
    def __init__(self):
        super().__init__('voice_interface')
        
        # Components
        self.wake_word = WakeWordDetector("hey robot")
        self.whisper = WhisperModel("base")
        self.parser = SemanticCommandParser()
        self.tts = TextToSpeech()
        self.executor = VoiceCommandExecutor()
        
        # State
        self.state = 'IDLE'  # IDLE, LISTENING, PROCESSING
        
        # Start audio stream
        self.start_audio_stream()
    
    def state_machine(self, audio_chunk):
        """Main voice interface state machine."""
        if self.state == 'IDLE':
            if self.wake_word.process_frame(audio_chunk):
                self.state = 'LISTENING'
                self.tts.speak_async("Yes?")
                self.audio_buffer = []
                self.start_silence_timer()
        
        elif self.state == 'LISTENING':
            self.audio_buffer.extend(audio_chunk)
            
            # Check for end of speech (silence detection)
            if self.detect_silence(audio_chunk):
                self.silence_count += 1
                if self.silence_count > self.silence_threshold:
                    self.state = 'PROCESSING'
                    self.process_command()
            else:
                self.silence_count = 0
        
        elif self.state == 'PROCESSING':
            pass  # Wait for processing to complete
    
    def process_command(self):
        """Process the recorded command."""
        audio = np.array(self.audio_buffer)
        
        # Transcribe
        segments, _ = self.whisper.transcribe(audio)
        text = " ".join([s.text for s in segments])
        
        if not text.strip():
            self.tts.speak("I didn't hear anything")
            self.state = 'IDLE'
            return
        
        self.get_logger().info(f"You said: {text}")
        
        # Parse and execute
        parsed = self.parser.parse(text)
        
        if parsed['confidence'] > 0.5:
            response = self.executor.execute(parsed)
            self.tts.speak(response)
        else:
            self.tts.speak("I'm not sure what you mean. Could you repeat that?")
        
        self.state = 'IDLE'
```

## 24.9 Summary

In this chapter, you learned:

- **Whisper integration**: Speech-to-text for robots
- **ROS 2 voice node**: Real-time audio processing
- **Wake word detection**: Activate on keyword
- **Command parsing**: Intent and entity extraction
- **Command execution**: Mapping speech to actions
- **Text-to-speech**: Robot voice responses

Voice enables natural interaction with humanoid robots. In the next chapter, we'll use LLMs for cognitive planning.

## Review Questions

1. What are the challenges of speech recognition on robots?
2. How does wake word detection improve the voice interface?
3. What is intent classification and why is it useful?
4. How do you extract entities from voice commands?
5. Why is text-to-speech important for voice interfaces?

## Hands-On Exercise

1. Set up Whisper for speech recognition
2. Create a ROS 2 node for voice commands
3. Implement wake word detection
4. Build an intent classifier for robot commands
5. Connect voice commands to robot actions
6. Add text-to-speech responses
