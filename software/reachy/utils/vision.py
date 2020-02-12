"""Utility module for vision."""

import cv2 as cv

from threading import Thread, Event, Lock


class BackgroundVideoCapture(object):
    """Wrapper on OpenCV VideoCapture object.

    This wrapper is reponsible for automatically polling image on the camera.
    This ensures that we can always access the most recent image.
    """

    def __init__(self, camera_index, resolution=(720, 960)):
        """Open video capture on the specified camera.

        Args:
            camera_index (int): index of the used camera (see OpenCV doc for details)
            resolution (int, int): desired resolution for the grabbed frame (the resolution must be compatible with the driver)

        Instantiating this object will automatically start the polling of image in background.
        """
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
        """Stop polling image and release the Video Capture."""
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
        """Retrieve the last grabbed image."""
        with self._lock:
            return True, self._img.copy()
