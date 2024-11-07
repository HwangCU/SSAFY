import cv2
import os
import numpy as np

path = os.getcwd()
print("내 workPath:", path)

# image resize and save.
# 1
img1 = cv2.imread("color_image.jpg")
img2 = cv2.imread("color_image.jpg", cv2.IMREAD_GRAYSCALE)
cv2.imwrite(path + "/gray_image.jpg", img2)

cv2.imshow("image1", img1)
cv2.imshow("image2", img2)

# 2
resized_img = cv2.resize(img1, [300, 300])
cv2.imshow("resized_img", resized_img)
cv2.imwrite(path + "/" + "resized_img.jpg", resized_img)

# 3
cropped_img = resized_img[50:200, 50:200]
cv2.imshow("cropped_img", cropped_img)
cv2.imwrite(path + "/" + "cropped_img.jpg", cropped_img)


# 4
# def get_coordinates(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:
#         print(f"좌표: ({x}, {y})")


# # 이미지 로드
# cv2.namedWindow("logo_img")
# cv2.setMouseCallback("logo_img", get_coordinates)

# # 이미지 표시
# while True:
#     cv2.imshow("logo_img", resized_img)
#     if cv2.waitKey(1) & 0xFF == ord("q"):
#         break

logo_img = resized_img[208:256, 58:83]
cv2.imshow("logo_img", logo_img)
cv2.imwrite(path + "/" + "logo_img.jpg", logo_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
