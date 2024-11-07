import tensorflow as tf
import cv2
import numpy as np

# MNIST 데이터셋 로드 및 모델 학습 준비
(x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()
x_train, x_test = x_train / 255.0, x_test / 255.0  # 정규화

# 간단한 CNN 모델 구성 및 학습
model = tf.keras.Sequential([
    tf.keras.layers.Flatten(input_shape=(28, 28)),
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(10, activation='softmax')
])

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

model.fit(x_train, y_train, epochs=15)
model.save("mnist_digit_recognition.h5")

# 실시간 숫자 인식
model = tf.keras.models.load_model("mnist_digit_recognition.h5")

# 웹캠 열기
cap = cv2.VideoCapture(4)

# 마우스 콜백을 위한 변수 초기화
drawing = False
x_start, y_start, x_end, y_end = -1, -1, -1, -1

# 마우스 콜백 함수
def draw_rectangle(event, x, y, flags, param):
    global x_start, y_start, x_end, y_end, drawing
    
    if event == cv2.EVENT_LBUTTONDOWN:  # 마우스 왼쪽 버튼을 누를 때
        drawing = True
        x_start, y_start = x, y

    elif event == cv2.EVENT_MOUSEMOVE:  # 마우스 이동 시
        if drawing:
            x_end, y_end = x, y

    elif event == cv2.EVENT_LBUTTONUP:  # 마우스 왼쪽 버튼을 뗄 때
        drawing = False
        x_end, y_end = x, y

# 이미지 전처리 함수
def preprocess_image(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_resized = cv2.resize(img_gray, (28, 28))
    img_resized = img_resized / 255.0  # 정규화
    img_resized = np.expand_dims(img_resized, axis=0)  # 모델 입력 형식에 맞추기
    return img_resized

# 마우스 콜백 함수 등록
cv2.namedWindow("Real-Time Digit Recognition")
cv2.setMouseCallback("Real-Time Digit Recognition", draw_rectangle)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 드래그 중일 때 사각형 그리기
    if x_start != -1 and y_start != -1 and x_end != -1 and y_end != -1:
        cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
    
    # 드래그 종료 후 영역 설정
    if not drawing and x_start != -1 and y_start != -1 and x_end != -1 and y_end != -1:
        roi = frame[min(y_start, y_end):max(y_start, y_end), min(x_start, x_end):max(x_start, x_end)]
        
        if roi.size > 0:
            # 이미지 전처리 및 예측
            processed_roi = preprocess_image(roi)
            prediction = model.predict(processed_roi)
            digit = np.argmax(prediction)

            # 결과 표시
            cv2.putText(frame, f"Digit: {digit}", (x_start, y_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    
    cv2.imshow("Real-Time Digit Recognition", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()