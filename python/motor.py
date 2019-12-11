# modified jetbot physical implementation
import atexit
import subprocess
import traitlets
from traitlets.config.configurable import Configurable


class Motor(Configurable):

    value = traitlets.Float()
    
    # config
    alpha = traitlets.Float(default_value=1.0).tag(config=True)
    beta = traitlets.Float(default_value=0.0).tag(config=True)

    def __init__(self, driver, channel, *args, **kwargs):
        super(Motor, self).__init__(*args, **kwargs)  # initializes traitlets

        self._motor = channel
        atexit.register(self._release)
        
    @traitlets.observe('value')
    def _observe_value(self, change):
        self._write_value(change['new'])

    def _write_value(self, value):
        """Sets motor value between [-3, 3] rad/s"""
        mapped_value = float(3 * (self.alpha * value + self.beta))
        subprocess.call(["motor_util", "-n", self._motor, "set", "--mode", "4", "--velocity", str(mapped_value)])

    def _release(self):
        """Stops motor by releasing control"""
        subprocess.call(["motor_util", "-n", self._motor, "set", "--mode", "0"])