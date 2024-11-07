import cv2
import torch

# FaceCheck 모델 불러오기
model_path = '/home/mj/yolov5/runs/train/exp9/weights/best.pt'
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)  # FaceCheck 모델 로드

# 비디오 파일 열기
video_path = '/home/mj/yolov5/video.mp4'
cap = cv2.VideoCapture(video_path)

# 비디오가 열리지 않으면 종료
if not cap.isOpened():
    print("Error: 비디오 파일을 열 수 없습니다.")
    exit()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break  # 비디오가 끝나면 종료

    # OpenCV의 BGR 포맷을 RGB로 변환
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 모델을 사용해 객체 검출 수행
    results = model(rgb_frame)

    # 결과를 원래 BGR 포맷의 프레임에 그리기
    results.render()  # 결과를 프레임에 그립니다
    frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)  # 다시 BGR로 변환

    # 화면에 출력
    cv2.imshow('Object Detection', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()