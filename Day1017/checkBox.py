import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QCheckBox, QPushButton, QVBoxLayout, QWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Checkbox Example")

        # 질문 레이블 생성
        question_label = QLabel("당신의 고향은 서울입니까?")

        # 체크박스 생성
        self.checkbox_yes = QCheckBox("YES")
        self.checkbox_no = QCheckBox("NO")

        # 버튼 생성 및 클릭 이벤트 연결
        self.button = QPushButton("Print Selection")
        self.button.clicked.connect(self.print_selection)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(question_label)
        layout.addWidget(self.checkbox_yes)
        layout.addWidget(self.checkbox_no)
        layout.addWidget(self.button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def print_selection(self):
        user = []
        if self.checkbox_yes.isChecked():
            user.append("YES")
        if self.checkbox_no.isChecked():
            user.append("NO")

        print("Selected values:", user)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())