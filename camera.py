import numpy as np
import picamera
import io
import time
from PIL import Image

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

with picamera.PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=1) as camera:
	stream = io.BytesIO()
	for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
		stream.seek(0)
		image = Image.open(stream).convert('RGB')
		start_time = time.monotonic()
		#results = detect_objects(interpreter, image, args.threshold)
		elapsed_ms = (time.monotonic() - start_time)*1000
		stream.seek(0)
		stream.truncate()

		print(np.mean(image))


