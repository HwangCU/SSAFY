import cv2
from mtcnn import MTCNN
import numpy as np

# MTCNN 얼굴 감지기 생성
detector = MTCNN()

# 웹캠 열기
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # MTCNN으로 얼굴 감지
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # MTCNN은 RGB 이미지를 사용
    faces = detector.detect_faces(rgb_frame)
    
    for face in faces:
        x, y, width, height = face['box']
        x2, y2 = x + width, y + height

        # 얼굴 위치에 사각형 표시
        cv2.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)
        
        # 감지된 얼굴의 좌표 표시
        cv2.putText(frame, f"x={x}, y={y}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 결과 표시
    cv2.imshow("Real-Time Face Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()