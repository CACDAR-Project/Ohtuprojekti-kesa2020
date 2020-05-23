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


def show_image(image_path):
    '''
    Load and convert an image from disk to grayscale. draw a line and show the result. OpenCV with pyplot
    '''
    if not check_image(image_path):
        return

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    plt.imshow(img, cmap='gray', interpolation='bicubic')
    plt.xticks([])
    plt.yticks([])
    plt.plot([200, 300, 400], [100, 200, 300], 'c', linewidth=5)
    detect_faces_from_image(image_path)
    plt.show()

    # cv2.imwrite('naturegray.png', img) #save image
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def show_video():
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    while True:
        ret, frame = cap.read(
        )  # ret is boolean indicating if we have any image returned, will be None if no image is returned
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # convert frame to grayscale. Opencv uses BGR-colors as oposed to RGB

        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def record_video():
    '''
    Show video and also save as xvid-encoded
    '''
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        out.write(frame)  # in colors
        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


def detect_faces_from_image(search_path):
    found_faces = 0
    img = cv2.imread(search_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

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
        cv2.putText(img, info, (0, 20), font, 1, (0, 0, 0), 1, cv2.LINE_4)
    elif found_faces == 1:
        info = f'{found_faces} person present'
        cv2.putText(img, info, (0, 20), font, 1, (0, 0, 0), 1, cv2.LINE_4)

    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def detect_faces_from_video():
    cap = cv2.VideoCapture(0)
    if not check_video(cap):
        return

    while True:
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]

            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh),
                              (0, 255, 0), 2)
        cv2.imshow('img', img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
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
    # show_image()
    # show_video()
    # record_video()
    # read_img()
    # detect_faces_from_video()
    config = Configuration()


def read_coordinates():
    pass


def save_coordinates():
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i',
                        '--image',
                        type=int,
                        choices=[0, 1, 2, 3, 4],
                        help='Show an image with face detection.')
    parser.add_argument('-v',
                        '--video',
                        help='Show video feed with face detection.',
                        action="store_true")
    parser.add_argument('-c', help='Read coordinates.', action="store_true")
    parser.add_argument('-s',
                        help='Save coordinates on file.',
                        action="store_true")
    args = parser.parse_args()
    if args.video:
        detect_faces_from_video()
    if args.image == 0:
        show_image('images/lion.jpg')
    elif args.image == 1:
        show_image('images/face_front.jpg.jpg')
    elif args.image == 2:
        show_image('images/nataliadyer.jpg')
    elif args.image == 3:
        show_image('images/nature.png')
    elif args.image == 4:
        show_image('images/six_people.jpg')
    else:
        show_image('images/six_people.jpg')
    main()
