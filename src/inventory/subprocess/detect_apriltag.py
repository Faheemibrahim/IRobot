#!/usr/bin/env python3

import sys
import os
import cv2
import numpy as np
from pupil_apriltags import Detector

# Directory to save detected images
DETECTED_TAGS_DIR = "/home/faheem/IRobot/src/inventory/detected_tags"
os.makedirs(DETECTED_TAGS_DIR, exist_ok=True)

def detect_and_save(image_path):
    """Loads an image, detects AprilTags, and saves if tags are found (in color)."""
    
    # Load image in color
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)  # Load in color mode
    if image is None:
        print("No image found")
        sys.exit(1)

    # Convert the color image to grayscale for AprilTag detection
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Initialize lightweight AprilTag detector
    detector = Detector(families='tag36h11')
    results = detector.detect(gray_image)

    if results:
        # Extract tag IDs
        tag_ids = "_".join(str(tag.tag_id) for tag in results)

        # Save the color image
        output_filename = os.path.join(DETECTED_TAGS_DIR, f"tag_{tag_ids}.png")
        cv2.imwrite(output_filename, image)  # Save the color image

        print("done")  # Signal to `capture.py` that processing is complete
    else:
        print("no tags found")  # Signal to `capture.py` if no tags detected

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No input image")
        sys.exit(1)

    detect_and_save(sys.argv[1])