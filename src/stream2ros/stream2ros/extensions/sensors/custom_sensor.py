import cv2
import importlib
import os

def init():
    # Dynamic load for custom sensor (user provides module.function)
    custom_module_str = os.environ.get('CUSTOM_SENSOR_MODULE', '')
    if custom_module_str:
        module_name, func_name = custom_module_str.rsplit('.', 1)
        custom_module = importlib.import_module(module_name)
        init_func = getattr(custom_module, func_name)
        cap = init_func()  # User-defined function returns cv2.VideoCapture or equivalent
        return cap
    else:
        raise ValueError('CUSTOM_SENSOR_MODULE env required for custom sensor')