import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QPushButton


class CheckBoxApp(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setWindowTitle("CheckBox Example")

        # 메인 레이아웃 설정
        vbox = QVBoxLayout()

        # 첫 번째 줄 체크박스들
        hbox1 = QHBoxLayout()
        self.cb1_1 = QCheckBox('1', self)
        self.cb1_2 = QCheckBox('2', self)
        self.cb1_3 = QCheckBox('3', self)
        hbox1.addWidget(self.cb1_1)
        hbox1.addWidget(self.cb1_2)
        hbox1.addWidget(self.cb1_3)
        vbox.addLayout(hbox1)

        # 두 번째 줄 체크박스들
        hbox2 = QHBoxLayout()
        self.cb2_1 = QCheckBox('4', self)
        self.cb2_2 = QCheckBox('5', self)
        self.cb2_3 = QCheckBox('6', self)
        hbox2.addWidget(self.cb2_1)
        hbox2.addWidget(self.cb2_2)
        hbox2.addWidget(self.cb2_3)
        vbox.addLayout(hbox2)

        # 출력 버튼
        self.btn = QPushButton('출력', self)
        self.btn.clicked.connect(self.print_checked)
        vbox.addWidget(self.btn)

        self.setLayout(vbox)

    def print_checked(self):
        checked_values = []
        if self.cb1_1.isChecked():
            checked_values.append(self.cb1_1.text())
        if self.cb1_2.isChecked():
            checked_values.append(self.cb1_2.text())
        if self.cb1_3.isChecked():
            checked_values.append(self.cb1_3.text())
        if self.cb2_1.isChecked():
            checked_values.append(self.cb2_1.text())
        if self.cb2_2.isChecked():
            checked_values.append(self.cb2_2.text())
        if self.cb2_3.isChecked():
            checked_values.append(self.cb2_3.text())
        
        print("Checked values:", checked_values)


def main():
    app = QApplication(sys.argv)
    ex = CheckBoxApp()
    ex.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
