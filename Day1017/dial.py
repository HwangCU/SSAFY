
import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QDial, QLabel
from PyQt6.QtCore import Qt

class DialApp(QWidget):
    def __init__(self):
        super().__init__()
        
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Dial Example")
        
        # 메인 레이아웃 설정
        vbox = QVBoxLayout()
        
        # 다이얼 생성
        self.dial = QDial(self)
        self.dial.setRange(-360, 360)
        self.dial.valueChanged.connect(self.update_label)
        vbox.addWidget(self.dial)
        
        # 값 표시 레이블 생성
        self.label = QLabel("0", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vbox.addWidget(self.label)
        
        # 레이아웃 설정
        self.setLayout(vbox)
        
    def update_label(self, value):
        self.label.setText(str(value))

def main():
    app = QApplication(sys.argv)
    ex = DialApp()
    ex.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()