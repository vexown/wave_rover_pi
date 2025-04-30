import cv2
import requests
import numpy as np
from ultralytics import YOLO 

# --- Configuration ---
# Replace with your Raspberry Pi's IP address
STREAM_URL = "https://192.168.1.191:8000/stream"
# Set to False if you are NOT using a self-signed certificate or have added it to your trust store
VERIFY_SSL = False
# --- End Configuration ---

# --- Object Detection Setup ---
# Load a pre-trained YOLOv5 model (e.g., yolov5s.pt, yolov5m.pt)
# 's' is small, 'm' is medium - choose based on your PC's capability
# The model will be downloaded automatically on first run if not present.
# To see available models, and for more info, see https://docs.ultralytics.com/ or https://github.com/ultralytics/ultralytics
#
# As of today (2025-04-30), the latest YOLO version is YOLO11 - https://docs.ultralytics.com/models/yolo11/
# List of YOLO11 Detection Models: yolo11n.pt yolo11s.pt yolo11m.pt yolo11l.pt yolo11x.pt (in order of size)
# Larger models are more accurate but slower.
YOLO_MODEL_NAME = "yolo11x.pt" 
model = YOLO(YOLO_MODEL_NAME)
print(f"{YOLO_MODEL_NAME} model loaded.")
# --- End Object Detection Setup ---

def receive_stream():
    """Connects to the MJPEG stream, performs object detection, and displays it."""
    print(f"Connecting to stream: {STREAM_URL}")
    try:
        # Use stream=True to avoid loading the entire (infinite) stream into memory
        # Use verify=False if using self-signed certificates (common for local setups)
        stream_request = requests.get(STREAM_URL, stream=True, verify=VERIFY_SSL)
        stream_request.raise_for_status() # Raise an exception for bad status codes (4xx or 5xx)

        bytes_buffer = bytes()
        for chunk in stream_request.iter_content(chunk_size=1024):
            bytes_buffer += chunk
            # MJPEG frames are typically separated by '--frame' boundary
            start = bytes_buffer.find(b'\xff\xd8') # Start of JPEG
            end = bytes_buffer.find(b'\xff\xd9')   # End of JPEG

            if start != -1 and end != -1 and end > start:
                jpg = bytes_buffer[start:end+2] # Extract the JPEG image data
                bytes_buffer = bytes_buffer[end+2:] # Remove the processed frame from buffer

                # Decode the JPEG data into an OpenCV image (NumPy array)
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    # --- Perform Object Detection ---
                    # Run inference on the frame
                    # stream=True is more efficient for video/streams
                    results = model(frame, stream=True, verbose=False) # Set verbose=False to reduce console output

                    # Process results generator
                    for result in results:
                        # Use the built-in plot() method to draw detections on the frame
                        frame = result.plot()
                    # --- End Object Detection ---


                    # Display the frame with detections
                    cv2.imshow('Pi Camera Stream with Detection', frame) # Updated window title

                    # Exit if 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    print("Failed to decode frame")

    except requests.exceptions.RequestException as e:
        print(f"Error connecting to stream: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Closing stream and window.")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Suppress insecure request warnings if not verifying SSL
    if not VERIFY_SSL:
        import urllib3
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    receive_stream()