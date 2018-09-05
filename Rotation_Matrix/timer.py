import time


class Timer:
    def __init__(self):
        pass

    def __enter__(self):
        self.start = time.time() * 1000.0
        return self

    def __exit__(self, *args):
        self.interval = time.time() * 1000.0 - self.start
