import cv2 as cv
from numpy import ndarray

def run():
    cap = cv.VideoCapture("test.mp4")
    ret, img = cap.read()
    print(type(img))

if __name__ == "__main__":
    run()
