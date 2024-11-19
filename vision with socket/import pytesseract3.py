import pytesseract
from PIL import Image

pytesseract.pytesseract.tesseract_cmd = "C:/Program Files/Tesseract-OCR/tesseract.exe"

image = Image.open('C:/Users/SSAFY/Downloads/test.png')
text = pytesseract.image_to_string(image)
print(text)