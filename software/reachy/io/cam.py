"""Wrapper around two RPI and the multi channel switch.

It relies on the use of the Multi_Adapter_Board_2Channel_uc444 for switching camera via the I2C and GPIO control.

See https://github.com/ArduCAM/RaspberryPi/tree/master/Multi_Camera_Adapter/Multi_Adapter_Board_2Channel_uc444

"""

import time
import cv2 as cv

from threading import Thread, Event, Lock
from gpiozero import DigitalOutputDevice
from smbus import SMBus

from ..error import CameraNotFoundError


class DualCamera(object):
    """Wrapper around two RPI and the multi channel switch."""

    def __init__(self, default_camera='right'):
        """Create a DualCamera obejct and set the default active."""
        self._pin17 = DigitalOutputDevice(17)
        self._pin4 = DigitalOutputDevice(4)
        self._bus = SMBus(1)

        self.set_active(default_camera)
        # Make sure, the camera is active before trying to access it.
        time.sleep(0.5)
        self.cap = BackgroundVideoCapture(0)

    def __exit__(self):
        """Automatically close the cam on exit."""
        self.close()

    def close(self):
        """Close the camera capture."""
        self.cap.close()
        self._pin17.close()
        self._pin4.close()
        self._bus.close()

    @property
    def active_side(self):
        """Get the active camera side."""
        return self._active

    def set_active(self, camera_side):
        """Set one of the camera active (left or right)."""
        if camera_side not in ('left', 'right'):
            raise ValueError('camera_side should be either "left" or "right"!')

        if camera_side == 'left':
            self._enable_left_camera()
        elif camera_side == 'right':
            self._enable_right_camera()

        self._active = camera_side

    def read(self):
        """Get the latest retrieved frame."""
        return self.cap.read()

    def _enable_left_camera(self):
        """Enable the left eye camera.

        See https://github.com/ArduCAM/RaspberryPi/tree/master/Multi_Camera_Adapter/Multi_Adapter_Board_2Channel_uc444
        """
        self._bus.write_byte_data(0x70, 0, 0x01)
        self._pin17.off()
        self._pin4.off()

    def _enable_right_camera(self):
        """Enable the left eye camera.

        See https://github.com/ArduCAM/RaspberryPi/tree/master/Multi_Camera_Adapter/Multi_Adapter_Board_2Channel_uc444
        """
        self._bus.write_byte_data(0x70, 0, 0x02)
        self._pin4.on()


class BackgroundVideoCapture(object):
    """Wrapper on OpenCV VideoCapture object.

    Args:
        camera_index (int): index of the used camera (see OpenCV doc for details)
        resolution (int, int): desired resolution for the grabbed frame (the resolution must be compatible with the driver)

    Instantiating this object will automatically start the polling of image in background.

    This wrapper is reponsible for automatically polling image on the camera.
    This ensures that we can always access the most recent image.
    """

    def __init__(self, camera_index, resolution=(720, 960)):
        """Open video capture on the specified camera."""
        self.cap = cv.VideoCapture(camera_index)

        if not self.cap.isOpened():
            raise CameraNotFoundError(
                message=f'Camera {camera_index} not found!',
                camera_id=camera_index,
            )

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
