import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt6.QtGui import QPainter, QPen
from PyQt6.QtCore import Qt, QPoint

class DrawingBoard(QWidget):
    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WidgetAttribute.WA_StaticContents)
        self.setFixedSize(800, 600)
        self.image = None
        self.last_point = QPoint()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.last_point = event.position().toPoint()
            self.image = self.grab()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton:
            painter = QPainter(self.image)
            pen = QPen(Qt.GlobalColor.black, 3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin)
            painter.setPen(pen)
            painter.drawLine(self.last_point, event.position().toPoint())
            self.last_point = event.position().toPoint()
            self.update()

    def paintEvent(self, event):
        canvas_painter = QPainter(self)
        if self.image:
            canvas_painter.drawImage(self.rect(), self.image.toImage(), self.image.rect())
        else:
            canvas_painter.fillRect(self.rect(), Qt.GlobalColor.white)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("PyQt6 Drawing Board")
        self.setGeometry(100, 100, 800, 600)
        
        self.drawing_board = DrawingBoard()
        self.setCentralWidget(self.drawing_board)

        self.show()

def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()