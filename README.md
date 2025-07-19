# Gaze Blink Interface

A Python based interface that allows users to interact with a screen using only their eye movements and blinks. Designed for hands free control, this system tracks gaze direction and detects eye blinks to trigger button selections in a UI. Developed as a university capstone project using a custom designed glasses setup with the camera.

# Features

* Real-time eye tracking from grayscale video input
* Directional dwell-based navigation (gaze direction held for 1 second triggers movement)
* GUI with 3×3 interactive button grid
* Designed for physically impaired users or alternative human-computer interaction

# How It Works

1. Camera Setup: The camera is mounted on a wearable glasses frame to capture a close-up of the left eye.
2. Frame Processing: The camera feed is cropped and processed to detect pupil position and determine gaze direction.
3. Navigation: If the user looks in one direction (e.g. right) and holds it for 1 second, the cursor moves accordingly on the GUI.
4. Selection: If the eye is closed for about 1 second (blink detection), the currently highlighted button is selected.

# Requirements:

* A Python environment
* A camera module (tested with OV9281 USB 3.0 grayscale camera)

# Packages

* numpy
* OpenCV
  
# Target Users

* Individuals with motor impairments
* Accessibility researchers and developers
* Human-computer interaction enthusiasts

# Acknowledgements

This project includes code adapted from the MIT-licensed repository by JEOresearch (2024).  
We thank the author for making it open source.
