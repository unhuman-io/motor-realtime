import subprocess
import re
import os
import time

def program(firmware_script = os.path.join(os.environ.get("OBOT_PATH"), "build/motor_aksim/load_motor_aksim.sh")):
    print("Programming " + firmware_script)
    result = subprocess.run([firmware_script], capture_output=True)
    match = re.findall(r"^File downloaded successfully", result.stdout.decode(), re.MULTILINE)
    if len(match) == 2:
        print("Successful download")
        time.sleep(3)
        return True
    else:
        print("Download failure")
        return False