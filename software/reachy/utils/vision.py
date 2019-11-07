import cv2 as cv

from queue import Queue
from threading import Thread, Event


class BackgroundVideoCapture(object):
    def __init__(self, camera_index):
        self.cap = cv.VideoCapture(camera_index)
        self.q = Queue(1)

        self.running = Event()

        self._t = Thread(target=self._read_loop)
        self._t.daemon = True
        self._t.start()

    def close(self):
        self.running.clear()

        if self._t.is_alive():
            self._t.join()

    def _read_loop(self):
        self.running.set()

        while self.running.is_set():
            b, img = self.cap.read()
            if b:
                if self.q.full():
                    self.q.get()
                self.q.put(img.copy())

    def read(self):
        if self.q.empty():
            return False, None

        return True, self.q.get()
