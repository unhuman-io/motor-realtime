import sys

class Logger(object):
    def __init__(self):
        self.terminal = sys.stdout
        self.log = open("log.dat", "a")

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
    
    def flush(self):
        self.log.flush()
        self.terminal.flush()

sys.stdout = Logger()
