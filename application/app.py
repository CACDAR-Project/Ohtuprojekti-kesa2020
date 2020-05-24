import cv2

from application.input.camera.camera import Camera
from application.output.screen.screen import Screen
from application.detection.qrdetect import QrDetect


def run_facedetection():
    pass


def run_something_else():
    pass


def detect_qr_codes(camera, output, qrdet):
    SCAN_FOR_CODES = True
    DRAW_RECTANGLES = True
    DRAW_POLY_BOUNDS = True
    DRAW_TEXTS = True

    while True:
        frame = camera.frameRGB()
        output.set_frame(frame)

        if not SCAN_FOR_CODES:
            continue

        qrdet.scan_frame(frame)

        if DRAW_RECTANGLES:
            rectangles = qrdet.get_rectangles_coords()
            output.add_rectangles(rectangles)
            #for r in rectangles:
            #    output.add_rectangle(*r)

        if DRAW_POLY_BOUNDS:
            polygons = qrdet.get_polygons()
            output.add_polygons(polygons)

        if DRAW_TEXTS:
            texts = qrdet.get_texts()
            output.add_texts(texts)

        output.draw()

        if cv2.waitKey(1) & 0xFF == ord('t'):
            output.toggle_gray()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #cv2.destroyAllWindows()


def run(conf, args):

    #conf = Configuration(load_settings_from_file=False)
    #args = handle_args(parser=argparse.ArgumentParser(), conf=conf)

    #print('We are up and running with the following args:')
    #print(args)

    # Run a simple webcam app that detects QR-codes,
    # Quit with q and toggle gray with t
    cam = Camera()
    out = Screen()
    qrdetector = QrDetect()
    detect_qr_codes(cam, out, qrdetector)


if __name__ == '__main__':
    run()
