import cv2
import os

burger_cascade = cv2.CascadeClassifier('burger_haarcascade.xml')


def detect():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print('No video stream detected')
        return

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('detections.avi', fourcc, 20.0,
                          (640, 480))

    while True:
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        results = burger_cascade.detectMultiScale(gray, 1.9, 8, minSize=(30, 40))

        for (x, y, w, h) in results:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]
            print('Upper left: ', x, ' Upper right: ', y, ' Lower left: ', w,
                  ' Lower right: ', h)

        cv2.imshow('img', img)
        out.write(img)  # in colors
        k = cv2.waitKey(30) & 0xff
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    detect()


main()
