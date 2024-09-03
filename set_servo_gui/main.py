import sys
from PyQt5.QtWidgets import QApplication

from servo_control import ServoControl


if __name__ == "__main__":
    app = QApplication(sys.argv)
    control = ServoControl()

    sys.exit(app.exec_())
