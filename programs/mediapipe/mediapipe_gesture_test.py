import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

import numpy as np

VisionRunningMode = mp.tasks.vision.RunningMode

base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options, running_mode=VisionRunningMode.IMAGE, num_hands=2, min_hand_detection_confidence=0.3)
recognizer = vision.GestureRecognizer.create_from_options(options)

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

cap = cv2.VideoCapture(0)   # カメラのID指定
if cap.isOpened():
    while True:
        # カメラから画像取得
        success, img = cap.read()
        if not success:
            continue

        # 検出処理の実行
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        recognition_result = recognizer.recognize(mp_image)
        if recognition_result.gestures:
          print(recognition_result.gestures)


        annotated_image = img.copy()
        hand_landmarks = recognition_result.hand_landmarks
        for hand_landmark in hand_landmarks:
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmark
            ])
            mp_drawing.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
        cv2.imshow("MediaPipe Hands", annotated_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q') or key == 0x1b:
            break

cap.release()
