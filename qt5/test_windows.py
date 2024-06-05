import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QStackedWidget, QSizePolicy, QShortcut
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt

class ImageDisplayApp(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Image Display App')

        # Create main layout
        vbox_main = QVBoxLayout()

        # Create stacked widget to manage pages
        self.stacked_widget = QStackedWidget(self)

        # Create labels for "Follow Mode" and "Inf Info"
        self.follow_mode_label = QLabel('Follow Mode Content', self)
        self.inf_info_label = QLabel('Inf Info Content', self)

        # Add labels to stacked widget
        self.stacked_widget.addWidget(self.follow_mode_label)
        self.stacked_widget.addWidget(self.inf_info_label)

        # Create buttons to switch between pages
        self.follow_mode_button = QPushButton('Follow Mode', self)
        self.inf_info_button = QPushButton('Inf Info', self)

        # Create shortcuts for switching pages using Tab key
        shortcut_follow_mode = QShortcut(QKeySequence(Qt.Key_Tab), self)
        shortcut_follow_mode.activated.connect(self.show_follow_mode_page)

        shortcut_inf_info = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_Tab), self)
        shortcut_inf_info.activated.connect(self.show_inf_info_page)

        # Add buttons to layout
        hbox_buttons = QHBoxLayout()
        hbox_buttons.addWidget(self.follow_mode_button)
        hbox_buttons.addWidget(self.inf_info_button)

        # Set button size policy to expand and add spacing between buttons
        self.follow_mode_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.inf_info_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        hbox_buttons.setSpacing(10)

        # Add stacked widget and buttons layout to main layout
        vbox_main.addLayout(hbox_buttons)
        vbox_main.addWidget(self.stacked_widget)

        self.setLayout(vbox_main)

        # Connect button clicks to page switching methods
        self.follow_mode_button.clicked.connect(self.show_follow_mode_page)
        self.inf_info_button.clicked.connect(self.show_inf_info_page)

        # Track whether buttons have been resized
        self.buttons_resized = False

    def show_follow_mode_page(self):
        self.stacked_widget.setCurrentIndex(0)  # Index of follow mode page
        if not self.buttons_resized:
            self.update_button_sizes()

    def show_inf_info_page(self):
        self.stacked_widget.setCurrentIndex(1)  # Index of inf info page
        if not self.buttons_resized:
            self.update_button_sizes()

    def update_button_sizes(self):
        for button in [self.follow_mode_button, self.inf_info_button]:
            font = button.font()
            font.setPointSize(font.pointSize() + 2)  # Increase font size by 2 points
            button.setFont(font)
        self.buttons_resized = True

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ImageDisplayApp()
    ex.show()
    sys.exit(app.exec_())
