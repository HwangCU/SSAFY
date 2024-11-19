
import cv2
import pyrealsense2 as rs
import pytesseract
import numpy as np

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # 마지막은 프레임 수

pipeline.start(config)


print("텍스트 인식 시작, Q누르면 종료")

try:
    while True:
        frames =pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()


        if not color_frame:
            continue
    
        color_image = np.asanyarray(color_frame.get_data())

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        text = pytesseract.image_to_string(gray_image, lang='eng')

        if text.strip():
            print(f"인식된 결과는 {text.strip()}입니다.")

        cv2.imshow('RealSense Camera', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("프로그램 종료")
            break
finally:
    cv2.destroyAllWindows()