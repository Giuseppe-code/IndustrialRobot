"""Camera client for the image stream produced by the Godot simulator."""

import cv2

from lib.dds.dds import DDS
from lib.dds.image_reader import ImageReader


class Camera:
    def __init__(self, host="localhost", port=4445, width=512, height=512):
        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.reader = ImageReader(self.host, self.port)

    def connect(self):
        if self.reader.sd.fileno() == -1:
            self.reader = ImageReader(self.host, self.port)
        self.reader.sd.settimeout(3.0)
        self.reader.connect()

    def read(self):
        return self.reader.read_image(self.width, self.height)

    def close(self):
        self.reader.sd.close()

    def reconnect(self):
        self.close()
        self.reader = ImageReader(self.host, self.port)
        self.connect()


def main():
    dds = DDS()
    dds.start()
    camera = Camera()
    camera.connect()

    try:
        while True:
            dds.publish("read_image", 1, DDS.DDS_TYPE_INT)
            image = camera.read()
            cv2.imshow("camera", image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.close()
        dds.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
