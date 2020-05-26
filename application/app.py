import cv2  # Only needed in the main loop for reading user input from console

from application.conf.configuration import Configuration
from application.input.camera.camera import Camera
from application.output.screen.screen import Screen
from application.detection.pyzbar import PyzbarDetector
from application.detection.opencvcascade import OpenCVCascade


def detection_loop(camera, output, codedet, facedet, eyedet):
    settings = Configuration.get_instance().settings
    WAIT_FOR_KEYPRESS = int(1000 / camera.get_fps())  # in ms

    while True:
        #frame = camera.frameRGB()
        # Use gray frame for detections
        frame, frame_gray = camera.frames()
        output.set_frame(frame)

        if settings['DETECT_FACES']:
            facedet.scan_frame(frame_gray)
            faces_coords = facedet.get_rectangles_coords()
            if settings['DRAW_FACES_RECTANGLES']:
                output.add_rectangles(faces_coords)

            if settings['DETECT_EYES']:
                # TODO: Slice frame and scan only faces for eyes.
                eyedet.scan_frame(frame_gray)
                eyes_coords = eyedet.get_rectangles_coords()
                if settings['DRAW_EYES_RECTANGLES']:
                    output.add_rectangles(eyes_coords)

        if settings['DETECT_CODES']:
            codedet.scan_frame(frame_gray)

            if settings['DRAW_CODES_RECTANGLES']:
                rectangles = codedet.get_rectangles_coords()
                output.add_rectangles(rectangles)

            if settings['DRAW_CODES_POLYGONS']:
                polygons = codedet.get_polygons()
                output.add_polygons(polygons)

            if settings['DRAW_CODES_TEXTS']:
                texts = codedet.get_texts()
                output.add_texts(texts)

        if settings['PRINT_DETECTIONS_CONSOLE']:
            # If detections are False, will print old info
            print('--------')
            print('Codes found: {}'.format(
                codedet.get_detected_codes_amount()))
            print('QR RECTANGLES:', rectangles)
            print('QR POLYGONS:', polygons)
            print('QR TEXTS:', texts)
            print('--------')
            print('Faces found: {}'.format(facedet.get_detections_amount()))
            print('FACES:', faces_coords)
            print('--------')
            print('Eyes found: {}'.format(eyedet.get_detections_amount()))
            print('EYES:', eyes_coords)
            print('--------')
            print()

        k = cv2.waitKey(WAIT_FOR_KEYPRESS)
        if k == ord('q'): break
        elif k == ord('t'): output.toggle_gray()
        elif k == ord('e'):
            settings['DETECT_EYES'] = not settings[
                'DETECT_EYES']  #Toggle eyes detection
        elif k == ord('f'):
            settings['DETECT_FACES'] = not settings[
                'DETECT_FACES']  #Toggle faces detection
        elif k == ord('c'):
            settings['DETECT_CODES'] = not settings[
                'DETECT_CODES']  #Toggle codes detection
        elif k == ord('p'):
            settings['PRINT_DETECTIONS_CONSOLE'] = not settings[
                'PRINT_DETECTIONS_CONSOLE']  # Toggle printing
        elif k == ord('-'):
            WAIT_FOR_KEYPRESS *= 2  # Decrease FPS
        elif k == ord('+'):
            WAIT_FOR_KEYPRESS //= 2  # Increase FPS

        output.draw()


def run():
    # Quit with q and toggle gray with t
    cam = Camera()
    out = Screen()

    qrdetector = PyzbarDetector(
        only_qr_codes=True)  # Set to False to scan for all supported symbols
    facedetector = OpenCVCascade(
        Configuration.get_instance().settings['DEFAULT_CASCADE_FACE'])
    eyedetector = OpenCVCascade(
        Configuration.get_instance().settings['DEFAULT_CASCADE_EYE'])

    # Use smaller resolution for improved performance
    cam.set_size(480, 270)
    #cam.print_video_info()
    #return
    #print('moo', cam.get_size())
    detection_loop(cam, out, qrdetector, facedetector, eyedetector)


if __name__ == '__main__':
    run()
