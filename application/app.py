import cv2

from application.input.camera.camera import Camera
from application.output.screen.screen import Screen
from application.detection.qrdetect import QrDetect
from application.detection.faces import FaceDetector


def detect_qr_codes(camera, output, qrdet, facedet, eyedet):
    SCAN_FOR_CODES = True
    DRAW_QR_RECTANGLES = True
    DRAW_QR_POLY_BOUNDS = True
    DRAW_QR_TEXTS = True

    SCAN_FOR_FACES = True
    SCAN_FOR_EYES = True

    PRINT_TO_CONSOLE = True
    while True:
        frame = camera.frameRGB()
        output.set_frame(frame)

        if SCAN_FOR_FACES:
            facedet.scan_frame(frame)

            faces_coords = facedet.get_rectangles_coords()
            output.add_rectangles(faces_coords)
            print('FACES:', faces_coords)
            if SCAN_FOR_EYES:
                # TODO: Slice frame and scan only faces for eyes.
                eyedet.scan_frame(frame)
                eyes_coords = eyedet.get_rectangles_coords()
                output.add_rectangles(eyes_coords)
                print('EYES:', eyes_coords)

            if PRINT_TO_CONSOLE:
                print('Faces found: {}'.format(
                    facedet.get_detections_amount()))
                print('Eyes found: {}'.format(eyedet.get_detections_amount()))

        if SCAN_FOR_CODES:
            qrdet.scan_frame(frame)

            if DRAW_QR_RECTANGLES:
                rectangles = qrdet.get_rectangles_coords()
                output.add_rectangles(rectangles)
                if PRINT_TO_CONSOLE:
                    print('QR RECTANGLES:', rectangles)
                #for r in rectangles:
                #    output.add_rectangle(*r)

            if DRAW_QR_POLY_BOUNDS:
                polygons = qrdet.get_polygons()
                output.add_polygons(polygons)
                if PRINT_TO_CONSOLE:
                    print('QR POLYGONS:', polygons)

            if DRAW_QR_TEXTS:
                texts = qrdet.get_texts()
                output.add_texts(texts)
                if PRINT_TO_CONSOLE:
                    print('QR TEXTS:', texts)

            if PRINT_TO_CONSOLE:
                print('Codes found: {}'.format(
                    qrdet.get_detected_codes_amount()))

        if PRINT_TO_CONSOLE:
            print()

        output.draw()

        if cv2.waitKey(1) & 0xFF == ord('t'):
            output.toggle_gray()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


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
    #self.face_cascade = cv2.CascadeClassifier('src/data/haarcascade_frontalface_default.xml')
    #self.eye_cascade = cv2.CascadeClassifier('src/data/haarcascade_eye.xml')
    facedetector = FaceDetector('src/data/haarcascade_frontalface_default.xml')
    eyedetector = FaceDetector('src/data/haarcascade_eye.xml')

    detect_qr_codes(cam, out, qrdetector, facedetector, eyedetector)


if __name__ == '__main__':
    run()
