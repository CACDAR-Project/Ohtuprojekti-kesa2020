# code from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html#goals
import argparse
import os
import cv2
import numpy as np
from matplotlib import pyplot as plt
from configuration import Configuration
from tools import filereaderwriter as fileio

font = cv2.FONT_HERSHEY_DUPLEX
face_cascade = cv2.CascadeClassifier(
    'src/data/haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('src/data/haarcascade_eye.xml')


def detect_faces_from_video():
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('src/saved_detections/output.avi', fourcc, 20.0,
                          (640, 480))

    while True:
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]
            print('Upper left: ', x, ' Upper right: ', y, ' Lower left: ', w,
                  ' Lower right: ', h)

            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh),
                              (0, 255, 0), 2)
        cv2.imshow('img', img)
        out.write(img)  # in colors
        k = cv2.waitKey(30) & 0xff
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def detect_faces_from_image(image_path):
    '''
    Load and convert an image from disk to grayscale. draw a line and show the result. OpenCV with pyplot
    '''
    found_faces = 0
    if not check_image(image_path):
        return

    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    plt.imshow(img, cmap='gray', interpolation='bicubic')
    plt.xticks([])
    plt.yticks([])
    plt.plot([200, 300, 400], [100, 200, 300], 'c', linewidth=5)
    plt.show()

    for (x, y, w, h) in faces:
        img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = img[y:y + h, x:x + w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        found_faces += 1
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0),
                          2)

    if found_faces > 1:
        info = f'{found_faces} people present'
    elif found_faces == 1:
        info = f'{found_faces} person present'

    savepath = os.getcwd() + '/src/saved_detections/' + 'faces.jpg'
    cv2.imwrite(savepath, img)
    cv2.imshow(info, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Check if picture exists
def check_image(given_path):
    if os.path.isfile(given_path):
        return True
    else:
        print('Cannot find image file.')
        return False


# Check if video exists
def check_video(video_cap):
    if video_cap.isOpened():
        return True
    else:
        print('No video stream detected')
        return False


def main():
    # exit all methods with pressing q
    # detect_faces_from_image()
    # show_video()
    # record_video()
    # read_img()
    # detect_faces_from_video()
    config = Configuration()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-i',
                       '--image',
                       type=int,
                       choices=[1, 2, 3],
                       help='Show an image with face detection.')
    group.add_argument('-v',
                       '--video',
                       help='Show video feed with face detection.',
                       action="store_true")
    args = parser.parse_args()
    if args.video:
        detect_faces_from_video()
    elif args.image:
        if args.image == 1:
            detect_faces_from_image('src/images/nature.png')
        elif args.image == 2:
            detect_faces_from_image('src/images/face_front.jpg')
        elif args.image == 3:
            detect_faces_from_image('src/images/six_people.jpg')
        else:
            detect_faces_from_image('src/images/six_people.jpg')
    else:
        print('Select -image with a number 1-3, or -video.')
    main()
