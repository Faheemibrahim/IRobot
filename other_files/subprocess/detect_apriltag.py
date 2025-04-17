#!/usr/bin/env python3

import sys
import os
import cv2
import numpy as np
from pupil_apriltags import Detector

# Use the same folder that `capture.py` searches in:
OUTPUT_DIR = "/home/faheem/IRobot/src/inventory/detected_tags"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def detect_and_save(image_path):
    
    """Loads an image, detects AprilTags, and saves a color copy if found."""
    
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        print("No image found")
        sys.exit(1)

    # Convert image to grayscale for detection
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Initialize the AprilTag detector
    detector = Detector(families='tag36h11')
    results = detector.detect(gray_image)

    if not results:
        print("no tags found")
        return

    # Example: If multiple tags are found, just pick the first:
    # (Alternatively, you could iterate over all tags if you want to handle more than one.)
    first_tag_id = results[0].tag_id

    # Save the color image using just the tag ID as filename: e.g. "0.png"
    output_filename = os.path.join(OUTPUT_DIR, f"{first_tag_id}.png")
    cv2.imwrite(output_filename, image)

    # Print the output in the format capture.py is expecting
    print(f"Detected Tag ID: {first_tag_id}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No input image")
        sys.exit(1)

    detect_and_save(sys.argv[1])