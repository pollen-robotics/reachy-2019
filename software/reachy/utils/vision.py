import cv2 as cv

from threading import Thread, Event, Lock


class BackgroundVideoCapture(object):
    def __init__(self, camera_index, resolution=(720, 960)):
        self.cap = cv.VideoCapture(camera_index)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, resolution[0])
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, resolution[1])

        self._lock = Lock()
        self.running = Event()

        self._img = None

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
                with self._lock:
                    self._img = img

    def read(self):
        with self._lock:
            return True, self._img.copy()
