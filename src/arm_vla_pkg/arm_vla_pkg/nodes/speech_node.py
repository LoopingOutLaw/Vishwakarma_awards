#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import time
import threading 

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
            self.recognizer.adjust_for_ambient_noise(source, duration=3.0)
        self.get_logger().info('Calibration complete. Ready to listen.')

        # --- REMOVED THE TIMER ---
        # self.listen_timer = self.create_timer(0.1, self.listen_loop)

        # --- ADDED A DEDICATED THREAD ---
        # This is a more stable way to handle a blocking task like listening
        self.listen_thread = threading.Thread(target=self.threaded_listen_loop, daemon=True)
        self.listen_thread.start()

    def threaded_listen_loop(self):
        # This loop will run forever in its own thread
        # It will block and wait for speech, which is what we want.
        while rclpy.ok():
            self.get_logger().info('Listening...')
            try:
                with self.microphone as source:
                    # Listen for a phrase, this will block until it hears speech
                    audio = self.recognizer.listen(source)
                
                # Recognize the audio
                self.get_logger().info('Recognizing speech...')
                recog_text = self.recognizer.recognize_google(audio).lower()
                
                self.get_logger().info(f'Heard user command: "{recog_text}"')
                
                # Publish the recognized text
                msg = String()
                msg.data = recog_text
                self.command_pub.publish(msg)

            except sr.UnknownValueError:
                # This happens if speech is not understood
                self.get_logger().warn('Could not understand audio, listening again.')
            except sr.RequestError as e:
                self.get_logger().error(f'Could not request results from Google; {e}')
            except Exception as e:
                self.get_logger().error(f'An unexpected error occurred: {e}')
                time.sleep(1) # Wait a second before retrying

def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechNode()
    try:
        rclpy.spin(speech_node)
    except KeyboardInterrupt:
        pass # Allow Ctrl+C to shut down
    finally:
        speech_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()