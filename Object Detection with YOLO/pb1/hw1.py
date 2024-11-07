import torch

# Model
model=torch.hub.load("ultralytics/yolov5","yolov5s")

# Images
img="https://ulytalytics.com/images/zidane.jpg"
img="/home/mj/yolov5"

# Inference
results=model(img)

# Results
# results.print()
# results.show()
# results.save()
# results.crop()
# print(results.pandas().xyxy[0])
