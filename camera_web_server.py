"""
Raspberry Pi Camera Streaming Server

This script sets up a web server on a Raspberry Pi to stream live video from a Camera Module 3 to a web browser using FastAPI.
The video is streamed in MJPEG format, which is viewable in most modern browsers. The server runs over HTTPS for secure access.

Key Features:
- Captures video frames from the Raspberry Pi Camera Module 3 using the Picamera2 library.
- Streams frames as JPEG images over HTTP using FastAPI's StreamingResponse.
- Corrects color channel ordering (RGB to BGR) to ensure accurate colors (e.g., orange walls appear orange).
- Uses asyncio for asynchronous frame generation to maintain smooth streaming.
- Configures the camera to start when the server launches and stop when it shuts down.

Requirements:
- Raspberry Pi with Camera Module 3.
- Python 3.11 or later.
- Installed libraries: `fastapi`, `uvicorn`, `picamera2`, `Pillow`, `numpy`.
  Install with: `pip install fastapi uvicorn picamera2 Pillow numpy`
  Also: 'pip install tflite_runtime opencv-python'
- SSL certificates for HTTPS (e.g., self-signed certs at `/home/blankmcu/certs/192.168.1.191.pem` and `/home/blankmcu/certs/192.168.1.191-key.pem`).
  (generated via mkcert but you could also use openssl or other tools)

How to Run:
1. Save this script as `camera_web_server.py` in your desired directory (e.g., `/home/blankmcu/Repos/wave_rover_pi`).
2. Ensure the SSL certificate and key files are in the specified paths.
3. Run the script: `python camera_web_server.py`
4. Open a web browser on a device on the same network and navigate to `https://192.168.1.191:8000/stream`.
   - Note: If using a self-signed certificate, you may need to accept a security warning in the browser.
5. To stop the server, press `Ctrl+C` in the terminal.

Troubleshooting:
- If the stream doesn’t load, check that the Raspberry Pi’s IP address is correct and the camera is connected.
- If colors appear incorrect, verify the color channel swap (`frame[:, :, ::-1]`) matches your camera’s output format.
- Ensure the SSL certificates are valid and accessible.
"""
import asyncio                                  # For asynchronous operations like frame generation and delays
from typing import AsyncGenerator               # Type hint for asynchronous generators (like generate_frames)
from contextlib import asynccontextmanager      # For creating asynchronous context managers (like lifespan)

from fastapi import FastAPI                     # The main FastAPI framework class
from fastapi.responses import StreamingResponse # For streaming responses like MJPEG video
from picamera2 import Picamera2                 # Library to control the Raspberry Pi Camera Module
from PIL import Image                           # Python Imaging Library (Pillow) for image manipulation (converting array to JPEG)
import io                                       # For handling in-memory binary streams (like the JPEG buffer)
import numpy as np                              # Library for numerical operations, used for handling image arrays
import cv2                                      # OpenCV for image processing (drawing boxes)
import tflite_runtime.interpreter as tflite     # TensorFlow Lite runtime interpreter

# --- Object Detection Setup ---
MODEL_PATH = "efficientdet_lite0_metadata.tflite"
LABEL_PATH = "coco_labels.txt"
CONFIDENCE_THRESHOLD = 0.5 # Only show detections with score >= 50%

# Load the labels
with open(LABEL_PATH, 'r') as f:
    labels = {i: line.strip() for i, line in enumerate(f.readlines())}

# Load the TFLite model and allocate tensors
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

# Get input and output tensor details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_height = input_details[0]['shape'][1]
input_width = input_details[0]['shape'][2]
# --- End Object Detection Setup ---

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Detailed explanation of the lifespan function:

    What is this function for?
    In FastAPI, the `lifespan` function is a special mechanism to run code
    right when the application starts up and right before it shuts down.
    This is perfect for managing resources that need to be initialized
    before the server can handle requests and cleaned up afterwards.
    In this case, we use it to start and stop the Raspberry Pi camera.

    What does `@asynccontextmanager` do?
    This is a "decorator" from Python's `contextlib`. It transforms the
    `lifespan` function (which uses `async def` and `yield`) into something
    called an "asynchronous context manager". Think of it as a manager
    that handles setup and teardown automatically.

    Why `async def`?
    `async def` marks the function as asynchronous. This means it can perform
    operations (like potentially waiting for the camera to start or stop)
    without blocking the entire application. FastAPI is built on asynchronous
    principles, so lifespan functions need to be `async`.

    How does FastAPI use this?
    When we create the FastAPI app like this: `app = FastAPI(lifespan=lifespan)`,
    FastAPI knows to use this `lifespan` function to manage the startup and
    shutdown procedures described above.
    """
    picam2.start()  # Initialize and start the camera. This line runs *before* the FastAPI application starts accepting web requests.
    print("Starting the camera...") 
    try:
        # This is the crucial part for a context manager. The `lifespan` function pauses here, and control is given back to FastAPI to run the
        # web server and handle requests (like streaming video). The code *after* `yield` won't run until the application starts shutting down.
        yield  # Allow the app to run while the camera is active
    finally:
        # This block guarantees that the code inside it will run *after* the `try` block finishes, regardless of whether the shutdown was normal 
        # or caused by an error.
        print("Stopping the camera...")
        picam2.stop()  # Stop the camera when the app shuts down. This line runs when the FastAPI application is shutting
                       # down (e.g., when you press Ctrl+C). It stops the camera, releasing the hardware resources.

####################################################################################
# Create a FastAPI application with the lifespan manager. 'app' and 'picam2' are defined globally so they are accessible
# when the script is imported as a module by uvicorn, and also available to the lifespan and endpoint functions.
app = FastAPI(lifespan=lifespan)

# Initialize and configure the Picamera2 instance globally
picam2 = Picamera2()
"""
Configure the camera using a preview configuration optimized for streaming.
create_preview_configuration() sets up sensible defaults for low-latency output.
Parameters explained:
    main (dict): Configures the main, high-resolution stream.
        "format" (str): Pixel format for the captured frames.
            - "RGB888": 3 bytes per pixel, Red-Green-Blue. Common for processing.
            - "BGR888": 3 bytes per pixel, Blue-Green-Red. Often needed for OpenCV.
            - "XRGB8888": 4 bytes per pixel, with an unused byte (X).
            - "YUV420": A planar format often used in video encoding. Less memory per pixel than RGB.
            - "MJPEG": Motion JPEG. The camera encodes directly to JPEG, useful for direct streaming if supported.
            (Note: Available formats depend on the specific camera sensor)
        "size" (tuple): Resolution (width, height) in pixels for the captured frames.
            - e.g., (640, 480), (1280, 720), (1920, 1080), (2304, 1296), (4608, 2592)
            (Note: Available resolutions depend on the specific camera sensor and selected format)
    (Other optional parameters like 'lores', 'raw', 'controls', 'buffer_count', 'queue' can also be passed
     to create_preview_configuration for more advanced setups, but 'main' is used here for basic streaming.)
"""
picam2.configure(
    picam2.create_preview_configuration(
        main={"format": "BGR888", "size": (1920, 1080)}
    )
)
####################################################################################


async def generate_frames() -> AsyncGenerator[bytes, None]:
    """
    Asynchronously generates video frames with object detection overlays:
    - Captures frames from the camera.
    - Flips the frame vertically and horizontally.
    - Prepares the frame for the TFLite model (resize, normalize).
    - Runs inference using the TFLite interpreter.
    - Draws bounding boxes and labels for detected objects above the confidence threshold.
    - Converts the frame with overlays to JPEG.
    - Yields frames in MJPEG format.
    """
    target_fps = 15 # Lower FPS slightly for detection overhead
    frame_duration = 1 / target_fps
    while True:
        start_time = asyncio.get_event_loop().time() # For FPS calculation

        # Capture a frame as a numpy array (shape: HxWx3 for BGR888)
        frame_original = picam2.capture_array()
        frame_height, frame_width, _ = frame_original.shape

        # Flip the frame vertically (upside down)
        frame_processed = np.flipud(frame_original)

        # Flip the frame horizontally (left-right)
        frame_processed = np.fliplr(frame_processed)

        # Ensure the array is C-contiguous for OpenCV drawing functions
        frame_processed = np.ascontiguousarray(frame_processed)

        # --- Object Detection ---
        # 1. Preprocess the frame for the model
        img_rgb = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2RGB) # Model expects RGB
        img_resized = cv2.resize(img_rgb, (input_width, input_height))
        input_data = np.expand_dims(img_resized, axis=0) # Add batch dimension

        # 2. Run inference
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # 3. Get detection results
        boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding boxes
        classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class IDs
        scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence scores
        # num_detections = interpreter.get_tensor(output_details[3]['index'])[0] # Number of detections (optional)

        # 4. Draw bounding boxes on the *processed* frame
        for i in range(len(scores)):
            if scores[i] >= CONFIDENCE_THRESHOLD:
                ymin, xmin, ymax, xmax = boxes[i]
                # Denormalize coordinates to original frame dimensions
                (left, right, top, bottom) = (xmin * frame_width, xmax * frame_width,
                                              ymin * frame_height, ymax * frame_height)

                # Draw bounding box
                cv2.rectangle(frame_processed, (int(left), int(top)), (int(right), int(bottom)), (0, 255, 0), 2) # Green box

                # Prepare label text
                object_name = labels.get(int(classes[i]), 'Unknown') # Look up class name
                label = f"{object_name}: {int(scores[i]*100)}%"

                # Draw label background
                label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                top = max(top, label_size[1]) # Ensure label doesn't go off screen
                cv2.rectangle(frame_processed, (int(left), int(top - label_size[1])),
                              (int(left + label_size[0]), int(top + base_line)), (0, 255, 0), cv2.FILLED)
                # Draw label text
                cv2.putText(frame_processed, label, (int(left), int(top)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2) # Black text

        # --- End Object Detection ---

        # Convert the processed frame (with drawings) to a PIL Image for JPEG encoding
        # Note: We are encoding frame_processed which has the drawings
        image = Image.fromarray(frame_processed) # Already BGR, PIL handles it

        # Save the image to a BytesIO buffer as JPEG
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG")
        frame_bytes = buffer.getvalue()

        # Yield the frame in MJPEG format
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")

        # Calculate time taken and sleep to maintain target FPS
        end_time = asyncio.get_event_loop().time()
        processing_time = end_time - start_time
        sleep_duration = max(0, frame_duration - processing_time)
        await asyncio.sleep(sleep_duration)


async def generate() -> AsyncGenerator[bytes, None]:
    """
    Wrapper function to iterate over frames from generate_frames().
    Required for FastAPI's StreamingResponse to handle the async generator.
    """
    async for frame in generate_frames():
        yield frame


@app.get("/stream")
async def video_stream():
    """
    FastAPI endpoint to stream video:
    - Accessible at `https://<your-pi-ip>:8000/stream`.
    - Returns a StreamingResponse with MJPEG content type.
    - The browser displays the stream as a continuous video.
    """
    return StreamingResponse(
        generate(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    import uvicorn
    """
    Entry point to run the FastAPI server:
    - Hosts the server on all network interfaces (`0.0.0.0`) at port 8000.
    - Uses HTTPS with the specified SSL certificate and key files.
    - Press `Ctrl+C` to stop the server.
    """
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        ssl_certfile="/home/blankmcu/certs/192.168.1.191.pem",
        ssl_keyfile="/home/blankmcu/certs/192.168.1.191-key.pem"
    )
