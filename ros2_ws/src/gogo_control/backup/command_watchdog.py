#!usr/bin/dev python3

import time


class CommandWatchdog:
    def __init__(self, timeout_sec: float):
        self.timeout_sec = timeout_sec
        self.last_kick_time = time.time()

    def kick(self):
        """Call this whenever a valid command is received."""
        self.last_kick_time = time.time()

    def is_timed_out(self) -> bool:
        """Returns True if commands have stopped."""
        return (time.time() - self.last_kick_time) > self.timeout_sec
