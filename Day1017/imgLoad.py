import sys
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt6.QtGui import QPixmap

class ImageWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        # 레이아웃 설정
        vbox = QVBoxLayout()

        # QLabel 생성
        label = QLabel(self)

        # 이미지 파일 경로 설정
        pixmap = QPixmap(r'C:\Users\SSAFY\Downloads\excited.gif')

        # QLabel에 pixmap 설정
        label.setPixmap(pixmap)

        # QLabel을 레이아웃에 추가
        vbox.addWidget(label)

        # 메인 레이아웃 설정
        self.setLayout(vbox)

        # 창 크기 조정
        self.resize(pixmap.width(), pixmap.height())

        # 창 제목 설정
        self.setWindowTitle('PyQt6 Image Example')
        self.show()

def main():
    app = QApplication(sys.argv)
    ex = ImageWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
