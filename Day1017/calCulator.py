import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton, QGridLayout
from PyQt6.QtCore import Qt

class Calculator(QWidget):
    def __init__(self):
        super().__init__()
        
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Calculator")

        # 전체 레이아웃 설정
        vbox = QVBoxLayout()

        # 디스플레이 설정
        self.display = QLineEdit(self)
        self.display.setReadOnly(True)
        self.display.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.display.setFixedHeight(50)
        vbox.addWidget(self.display)

        # 버튼 레이아웃 설정
        grid = QGridLayout()

        buttons = [
            ('7', 0, 0), ('8', 0, 1), ('9', 0, 2), ('/', 0, 3),
            ('4', 1, 0), ('5', 1, 1), ('6', 1, 2), ('*', 1, 3),
            ('1', 2, 0), ('2', 2, 1), ('3', 2, 2), ('-', 2, 3),
            ('0', 3, 0), ('.', 3, 1), ('=', 3, 2), ('+', 3, 3),
        ]

        for (text, row, col) in buttons:
            button = QPushButton(text, self)
            button.setFixedSize(40, 40)
            button.clicked.connect(self.on_click)
            grid.addWidget(button, row, col)

        vbox.addLayout(grid)
        self.setLayout(vbox)

    def on_click(self):
        sender = self.sender()
        text = sender.text()

        if text == '=':
            try:
                result = str(eval(self.display.text()))
                self.display.setText(result)
            except Exception as e:
                self.display.setText('Error')
        elif text in {'+', '-', '*', '/'}:
            self.display.setText(self.display.text() + ' ' + text + ' ')
        else:
            self.display.setText(self.display.text() + text)

def main():
    app = QApplication(sys.argv)
    calc = Calculator()
    calc.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()