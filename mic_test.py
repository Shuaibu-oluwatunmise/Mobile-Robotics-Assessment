from vosk import Model, KaldiRecognizer
import pyaudio
import json

model = Model("/home/ros/vosk-model-small-en-us-0.15")  # Adjust path if needed
recognizer = KaldiRecognizer(model, 16000)

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,
                input=True, frames_per_buffer=8192)
stream.start_stream()

print("🎙️ Speak now (Ctrl+C to exit)...")

try:
    while True:
        data = stream.read(4096)
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            print("You said:", result['text'])
except KeyboardInterrupt:
    print("\nDone.")

