---
title: Voice-to-Action with OpenAI Whisper
description: Implementing speech recognition and command interpretation using OpenAI Whisper
sidebar_position: 2
---

# Voice-to-Action with OpenAI Whisper

## Introduction to OpenAI Whisper

OpenAI Whisper is a robust automatic speech recognition (ASR) system trained on a vast dataset of diverse audio samples. Its multilingual capabilities and accuracy make it ideal for robotics applications where natural language interaction is required. In this section, we'll integrate Whisper into our ROS 2 system to enable voice-controlled humanoid robots.

## Setting Up Whisper in ROS 2

To begin, we need to install the Whisper library and its dependencies in our ROS 2 environment:

```bash
pip install openai-whisper
pip install faster-whisper  # For faster inference
```

## Audio Capture and Preprocessing

For real-time voice-to-action systems, we need to continuously capture audio and process it in chunks. Here's a basic ROS 2 node structure for audio capture:

```python
import rclpy
from rclpy.node import Node
import whisper
import pyaudio
import numpy as np
from std_msgs.msg import String

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # Initialize Whisper model
        self.model = whisper.load_model("base.en")  # or "small.en", "medium.en", etc.
        
        # Audio parameters
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Create publisher for recognized commands
        self.command_publisher = self.create_publisher(String, 'voice_commands', 10)
        
        # Start audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        
        # Timer to periodically process audio
        self.timer = self.create_timer(0.1, self.process_audio)
        
    def process_audio(self):
        # Read audio chunk
        data = self.stream.read(self.chunk_size)
        
        # Convert to numpy array and normalize
        audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        
        # Process with Whisper (this is simplified; real implementation would buffer chunks)
        # For continuous recognition, you'd need to accumulate audio over time
        # and then pass it to Whisper for transcription
        
        # For demonstration, we'll skip the actual Whisper call here
        # and just publish a dummy command
        msg = String()
        msg.data = "move forward"
        self.command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-Time Speech Recognition Pipeline

For real-time speech recognition, we need to implement a buffering mechanism that accumulates audio over time and processes it when enough context is available:

```python
import queue
import threading
import time

class ContinuousWhisperNode(Node):
    def __init__(self):
        super().__init__('continuous_whisper_node')
        
        # Initialize Whisper model
        self.model = whisper.load_model("base.en")
        
        # Audio parameters
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.buffer_duration = 5  # seconds of audio to process at once
        
        # Audio buffer
        self.audio_buffer = []
        self.buffer_lock = threading.Lock()
        
        # Queue for processed audio
        self.audio_queue = queue.Queue()
        
        # Publisher for recognized text
        self.text_publisher = self.create_publisher(String, 'recognized_text', 10)
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        
        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        # Start processing timer
        self.process_timer = self.create_timer(0.1, self.check_and_process)
        
    def capture_audio(self):
        while True:
            data = self.stream.read(self.chunk_size)
            audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            
            with self.buffer_lock:
                self.audio_buffer.extend(audio_data)
                
                # If buffer is large enough, process it
                if len(self.audio_buffer) >= self.rate * self.buffer_duration:
                    # Extract the last N seconds of audio
                    samples_needed = int(self.rate * self.buffer_duration)
                    audio_segment = np.array(self.audio_buffer[-samples_needed:])
                    self.audio_buffer = self.audio_buffer[-int(self.rate * 1):]  # Keep last 1 second overlap
                    
                    # Put the segment in the queue for processing
                    self.audio_queue.put(audio_segment)
    
    def check_and_process(self):
        try:
            # Get audio from queue if available
            audio_segment = self.audio_queue.get_nowait()
            
            # Process with Whisper
            result = self.model.transcribe(audio_segment)
            text = result['text'].strip()
            
            if text:  # If we got some text
                msg = String()
                msg.data = text
                self.text_publisher.publish(msg)
                
        except queue.Empty:
            pass  # No audio to process right now

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousWhisperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Command Interpretation

Once we have transcribed text, we need to interpret it into actionable commands for the robot. This can be done using a combination of pattern matching and semantic understanding:

```python
import re
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandInterpreterNode(Node):
    def __init__(self):
        super().__init__('command_interpreter_node')
        
        # Subscriber for recognized text
        self.text_subscriber = self.create_subscription(
            String,
            'recognized_text',
            self.text_callback,
            10
        )
        
        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def text_callback(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f'Received command: {text}')
        
        # Simple pattern matching for commands
        if 'forward' in text or 'go ahead' in text:
            self.move_forward()
        elif 'backward' in text or 'go back' in text:
            self.move_backward()
        elif 'turn left' in text or 'rotate left' in text:
            self.turn_left()
        elif 'turn right' in text or 'rotate right' in text:
            self.turn_right()
        elif 'stop' in text or 'halt' in text:
            self.stop_robot()
        else:
            self.get_logger().info(f'Unknown command: {text}')
    
    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Adjust speed as needed
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Moving forward')
    
    def move_backward(self):
        twist_msg = Twist()
        twist_msg.linear.x = -0.5  # Adjust speed as needed
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Moving backward')
    
    def turn_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Adjust rotation speed as needed
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Turning left')
    
    def turn_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -0.5  # Adjust rotation speed as needed
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Turning right')
    
    def stop_robot(self):
        twist_msg = Twist()
        # Zero velocities
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Stopping robot')

def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Optimizing Whisper Performance

To optimize Whisper for real-time robotics applications:

1. **Model Selection**: Choose a smaller model ("tiny" or "base") for faster inference at the cost of some accuracy
2. **Hardware Acceleration**: Use GPU acceleration when available
3. **Preprocessing**: Filter audio to focus on human speech frequencies
4. **VAD Integration**: Use a Voice Activity Detection (VAD) system to only process when someone is speaking

## Conclusion

With OpenAI Whisper integrated into our ROS 2 system, we now have the foundation for voice-controlled humanoid robots. The next step is to enhance this with cognitive planning using Large Language Models (LLMs) to interpret more complex commands and generate sophisticated action sequences.