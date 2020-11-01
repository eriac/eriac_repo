#!/usr/bin/python
import cv2

cap = cv2.VideoCapture("videotestsrc ! videoconvert ! appsink")

while True:
    ret, img = cap.read()
    if not ret:
        break

    cv2.imshow("",img)
    key = cv2.waitKey(1)
    if key==27: #[esc] key
        break
