import importlib
import os

def init():
    # Dynamic load for custom ML (user provides module.function that returns processor)
    custom_module_str = os.environ.get('CUSTOM_ML_MODULE', '')
    if custom_module_str:
        module_name, func_name = custom_module_str.rsplit('.', 1)
        custom_module = importlib.import_module(module_name)
        init_func = getattr(custom_module, func_name)
        processor = init_func()  # User-defined init returns callable processor(frame) -> frame
        return processor
    else:
        raise ValueError('CUSTOM_ML_MODULE env required for custom ML')

    # Stub example if no dynamic:
    # def processor(frame):
    #     # Your custom ML here, e.g., load model and apply
    #     return frame  # Placeholder
    # return processor