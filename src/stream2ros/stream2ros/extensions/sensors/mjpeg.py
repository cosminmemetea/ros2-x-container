import cv2
import os

def init():
    url = os.environ.get('STREAM_URL', 'http://host.docker.internal:8080/source_0')
    cap = cv2.VideoCapture(url)
    return cap