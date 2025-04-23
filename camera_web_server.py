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
- SSL certificates for HTTPS (e.g., self-signed certs at `/home/blankmcu/certs/192.168.1.191.pem` and `/home/blankmcu/certs/192.168.1.191-key.pem`).

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

import asyncio
from typing import AsyncGenerator
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from picamera2 import Picamera2
from PIL import Image
import io
import numpy as np

# Define a lifespan context manager to handle camera startup and shutdown
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manages the camera's lifecycle:
    - Starts the camera when the FastAPI app starts.
    - Ensures the camera is stopped when the app shuts down to free resources.
    """
    picam2.start()  # Initialize and start the camera
    try:
        yield  # Allow the app to run while the camera is active
    finally:
        picam2.stop()  # Stop the camera when the app shuts down

# Create a FastAPI application with the lifespan manager
app = FastAPI(lifespan=lifespan)

# Initialize and configure the Picamera2 instance
picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
)
"""
Camera configuration:
- Uses `RGB888` format (3-channel RGB, 8 bits per channel) for capturing frames.
- Sets resolution to 640x480 pixels, suitable for streaming with low latency.
- `create_preview_configuration` optimizes for real-time display/streaming.
"""

async def generate_frames() -> AsyncGenerator[bytes, None]:
    """
    Asynchronously generates video frames for streaming:
    - Captures frames from the camera.
    - Converts frames to JPEG images with correct color ordering.
    - Yields frames in MJPEG format for the browser.
    """
    while True:
        # Capture a frame as a numpy array (shape: 480x640x3 for RGB888)
        frame = picam2.capture_array()
        
        # Swap color channels (RGB to BGR) to correct color display
        # Without this, colors may appear incorrect (e.g., orange walls look blue)
        frame = frame[:, :, ::-1]
        
        # Convert the numpy array to a PIL Image for JPEG encoding
        image = Image.fromarray(frame)
        
        # Save the image to a BytesIO buffer as JPEG
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG")
        frame_bytes = buffer.getvalue()
        
        # Yield the frame in MJPEG format with appropriate headers
        # The `--frame` boundary and `Content-Type` are required for MJPEG streaming
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
        
        # Pause briefly to achieve ~10 FPS (0.1 seconds per frame)
        # Adjust this value to change the frame rate (e.g., 0.05 for 20 FPS)
        await asyncio.sleep(0.1)

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
