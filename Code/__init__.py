import os

print(f"__init__.py is being executed from: {os.path.dirname(os.path.abspath(__file__))}")
from Code.camera import setup_camera
print("Imported setup_camera successfully")

__all__ = ['setup_camera']
print("__all__ set to include setup_camera")