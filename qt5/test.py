import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt

class Window1(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Window 1')
        layout = QVBoxLayout()
        label = QLabel('This is Window 1')
        layout.addWidget(label)
        self.setLayout(layout)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Tab:
            self.hide()
            window2.show()

class Window2(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Window 2')
        layout = QVBoxLayout()
        label = QLabel('This is Window 2')
        layout.addWidget(label)
        self.setLayout(layout)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Tab:
            self.hide()
            window1.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    window1 = Window1()
    window2 = Window2()

    window1.show()

    sys.exit(app.exec_())
