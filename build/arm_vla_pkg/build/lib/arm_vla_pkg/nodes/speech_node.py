#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import time

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self.get_logger().info('Speech Node (Ears) is running...')
        
        # Publisher for the user's text command
        self.command_pub = self.create_publisher(String, '/user_command', 10)
        
        # Set up the speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise once at the start
        self.get_logger().info('Calibrating for ambient noise... please be quiet.')
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
        self.get_logger().info('Calibration complete. Listening...')

        # Create a timer to run the listening loop
        self.listen_timer = self.create_timer(0.1, self.listen_loop)

    def listen_loop(self):
        # This is a non-blocking loop
        try:
            with self.microphone as source:
                # Listen for a short phrase
                audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=4)
            
            # Recognize the audio
            self.get_logger().info('Recognizing speech...')
            recog_text = self.recognizer.recognize_google(audio).lower()
            
            self.get_logger().info(f'Heard user command: "{recog_text}"')
            
            # Publish the recognized text
            msg = String()
            msg.data = recog_text
            self.command_pub.publish(msg)

        except sr.UnknownValueError:
            # This happens if no speech is heard, which is normal
            self.get_logger().debug('No speech detected.')
        except sr.RequestError as e:
            self.get_logger().error(f'Could not request results from Google; {e}')
        except sr.WaitTimeoutError:
            # This happens if no audio is detected in the timeout, which is normal
            self.get_logger().debug('Listen timeout, restarting...')

def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechNode()
    rclpy.spin(speech_node)
    speech_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()