import cv2
import os

def init():
    device_index = int(os.environ.get('WEBCAM_INDEX', 0))
    cap = cv2.VideoCapture(device_index)
    return cap