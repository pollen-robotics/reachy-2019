"""Wrapper around two RPI and the multi channel switch.

It relies on the use of the Multi_Adapter_Board_2Channel_uc444 for switching camera via the I2C and GPIO control.

See https://github.com/ArduCAM/RaspberryPi/tree/master/Multi_Camera_Adapter/Multi_Adapter_Board_2Channel_uc444

"""

import cv2 as cv

from threading import Thread, Event, Lock

from ..error import CameraNotFoundError


class BackgroundVideoCapture(object):
    """Wrapper on OpenCV VideoCapture object.

    Args:
        camera_index (int): index of the used camera (see OpenCV doc for details)
        resolution (int, int): desired resolution for the grabbed frame (the resolution must be compatible with the driver)

    Instantiating this object will automatically start the polling of image in background.

    This wrapper is reponsible for automatically polling image on the camera.
    This ensures that we can always access the most recent image.
    """

    def __init__(self, camera_index, resolution=(600, 800), exposure=600):
        """Open video capture on the specified camera."""
        self.cap = cv.VideoCapture(camera_index)

        if not self.cap.isOpened():
            raise CameraNotFoundError(
                message=f'Camera {camera_index} not found!',
                camera_id=camera_index,
            )

        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, resolution[0])
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, resolution[1])
        self.cap.set(cv.CAP_PROP_EXPOSURE, exposure)
        self.cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)

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

        self.cap.release()

    def _read_loop(self):
        self.running.set()

        while self.running.is_set():
            b, img = self.cap.read()

            if b:
                with self._lock:
                    self._img = img.copy()

    def read(self):
        """Retrieve the last grabbed image."""
        with self._lock:
            return self._img is not None, self._img
