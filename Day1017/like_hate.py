import sys
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QPushButton, QHBoxLayout
from PyQt6.QtGui import QPixmap

class ImageWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        # 메인 레이아웃 설정
        vbox = QVBoxLayout()

        # QLabel 생성
        label = QLabel(self)

        # 이미지 파일 경로 설정
        pixmap = QPixmap(r'C:\Users\lemon\OneDrive\사진\사진1.png')

        # QLabel에 pixmap 설정
        label.setPixmap(pixmap)

        # QLabel을 메인 레이아웃에 추가
        vbox.addWidget(label)

        # 버튼 레이아웃 생성
        hbox = QHBoxLayout()

        # QPushButton 생성 및 레이아웃에 추가
        button1 = QPushButton('좋아요', self)
        button2 = QPushButton('싫어요', self)

        hbox.addWidget(button1)
        hbox.addWidget(button2)

        # 버튼 레이아웃을 메인 레이아웃에 추가
        vbox.addLayout(hbox)

        # 메인 레이아웃 설정
        self.setLayout(vbox)

        # 창 크기 조정
        self.resize(pixmap.width(), pixmap.height() + 50)  # 버튼이 포함되도록 크기 조정

        # 창 제목 설정
        self.setWindowTitle('PyQt6 Image Example with Buttons')
        self.show()

def main():
    app = QApplication(sys.argv)
    ex = ImageWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()