"""Mirror mission console output to a durable text log."""

import os
import sys
import threading
from contextlib import contextmanager
from pathlib import Path


class _Tee:
    def __init__(self, original, stream, lock):
        self.original = original
        self.stream = stream
        self.lock = lock

    def write(self, value):
        if value:
            with self.lock:
                self.stream.write(value)
            self.original.write(value)

    def flush(self):
        with self.lock:
            self.stream.flush()
        self.original.flush()


@contextmanager
def capture_console(path):
    """Keep stdout/stderr visible while saving both in the mission directory."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    stream = path.open('a', encoding='utf-8', buffering=1)
    lock = threading.Lock()
    old_stdout, old_stderr = sys.stdout, sys.stderr
    sys.stdout = _Tee(old_stdout, stream, lock)
    sys.stderr = _Tee(old_stderr, stream, lock)
    try:
        yield path
    finally:
        sys.stdout, sys.stderr = old_stdout, old_stderr
        with lock:
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()
