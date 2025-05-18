#!/usr/bin/env python3

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from vosk import Model, KaldiRecognizer
import pyaudio
import json

def listen_for_command(recognizer, stream):
    print("🎙️ Listening for a destination command (e.g., 'home', 'kitchen')...")
    while True:
        data = stream.read(4096)
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get('text', '').lower()
            print(f"Heard: {text}")
            return text

def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot4Navigator()

    # Load Vosk model
    model_path = "/home/ros/Downloads/vosk-model-small-en-us-0.15"
    model = Model(model_path)
    recognizer = KaldiRecognizer(model, 16000)

    # Setup microphone
    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=8192
    )
    stream.start_stream()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([1.4719, -1.5888, -0.0014], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # Define destinations with keywords
    destinations = [
        {
            'keywords': ['home'],
            'pose': navigator.getPoseStamped([1.3560, -1.2528, -0.0013], TurtleBot4Directions.EAST)
        },
        {
            'keywords': ['kitchen', 'the kitchen'],
            'pose': navigator.getPoseStamped([3.2812, -0.2662, -0.0014], TurtleBot4Directions.EAST)
        },
        {
            'keywords': ['dining', 'dining area', 'dining room'],
            'pose': navigator.getPoseStamped([3.5629, -3.0221, -0.0013], TurtleBot4Directions.NORTH)
        },
        {
            'keywords': ['sitting', 'sitting room', 'lounge'],
            'pose': navigator.getPoseStamped([-0.2123, 0.8335, -0.0014], TurtleBot4Directions.NORTH_WEST)
        },
        {
            'keywords': ['exit', 'stop'],
            'pose': None
        }
    ]

    navigator.info("🗣️ Voice control active. Say a destination to navigate.")

    try:
        while rclpy.ok():
            spoken_text = listen_for_command(recognizer, stream)
            matched = False

            for destination in destinations:
                for keyword in destination['keywords']:
                    if keyword in spoken_text:
                        matched = True
                        if 'exit' in destination['keywords']:
                            navigator.info("Exiting voice navigation.")
                            rclpy.shutdown()
                            return
                        navigator.info(f"Navigating to: {keyword}")
                        navigator.startToPose(destination['pose'])
                        break
                if matched:
                    break

            if not matched:
                navigator.info("Didn't catch a valid destination. Try again.")

    except KeyboardInterrupt:
        print("\nInterrupted. Exiting.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

