import subprocess
import sys

if __name__ == "__main__":
    # Runs the nanopositioner_gui.py script as an independent process
    subprocess.Popen([sys.executable, "nanopositioner_gui.py"])
    # Runs the camera.py script as an independent process
    subprocess.Popen([sys.executable, "camera.py"])