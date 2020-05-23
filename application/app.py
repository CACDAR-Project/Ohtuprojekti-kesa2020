#import argparse
import cv2

#from application.conf.configuration import Configuration
from application.camera.camera import Camera
from application.output.screen.draw import Draw


def run_facedetection():
    pass


def run_something_else():
    pass


def show_video(camera, output):
    while True:
        frame = camera.frameRGB()
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('frame', gray)
        output.set_frame(frame)
        output.draw()
        if cv2.waitKey(2) & 0xFF == ord('t'):
            output.toggle_gray()
        elif cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #cv2.destroyAllWindows()


def run(conf, args):

    #conf = Configuration(load_settings_from_file=False)
    #args = handle_args(parser=argparse.ArgumentParser(), conf=conf)

    #print('We are up and running with the following args:')
    #print(args)

    # Run a simple webcam app, quit with q and toggle gray with t
    cam = Camera()
    out = Draw()
    show_video(cam, out)


if __name__ == '__main__':
    run()
