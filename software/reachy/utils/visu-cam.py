import cv2 as cv
import sys

cap = cv.VideoCapture(int(sys.argv[1]))
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))

first = True

while True:
    b, img = cap.read()

    if b:
        if first:
            print(img.shape)
            first = False

        cv.imshow('live', img)

    cv.waitKey(20)
