from threading import Lock, Timer

class Watchdog:
    """Threaded watchdog that calls a callback if not kicked in time."""

    def __init__(self, timeout_sec: float, expired_callback):
        self.timeout_sec = timeout_sec
        self.expired_callback = expired_callback
        self._lock = Lock()
        self._timer = None
        self._active = True
        self.kick()  # start immediately

    def _timer_callback(self):
        with self._lock:
            if self._active:
                self.expired_callback()

    def kick(self):
        """Reset the watchdog timer."""
        with self._lock:
            if self._timer:
                self._timer.cancel()
            if self._active:
                self._timer = Timer(self.timeout_sec, self._timer_callback)
                self._timer.start()

    def cancel(self):
        """Stop the watchdog completely."""
        with self._lock:
            self._active = False
            if self._timer:
                self._timer.cancel()
                self._timer = None