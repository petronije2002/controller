import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QFormLayout, QSpinBox, QDoubleSpinBox, QPushButton, QLineEdit
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Controller")
        self.setGeometry(100, 100, 800, 600)

        # Initialize Serial Connection
        self.ser = None

        # Set up main tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        
        self.tabs.addTab(self.create_pid_tab(), "PID Settings")
        self.tabs.addTab(self.create_motor_settings_tab(), "Motor Settings")

        # Set up chart for system response
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position/Velocity')
        self.ax.set_title('System Response')
        self.tabs.addTab(self.create_chart_tab(), "System Response")
        
        # Timer to update chart periodically
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_chart)
        self.timer.start(1000)  # Update every second

        # Initialize variables for chart data
        self.time_data = []
        self.position_data = []

    def create_pid_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()

        # Test section for PID
        test_layout = QFormLayout()
        
        # Desired Position input
        self.desired_position_input = QSpinBox()
        self.desired_position_input.setRange(0, 1000)  # Set range for position
        self.desired_position_input.setValue(100)  # Default value

        # Desired Velocity input
        self.desired_velocity_input = QDoubleSpinBox()
        self.desired_velocity_input.setRange(-1000, 1000)  # Set range for velocity
        self.desired_velocity_input.setValue(50.0)  # Default value
        self.desired_velocity_input.setSuffix(" m/s")  # Add suffix

        # Test button
        test_button = QPushButton("Send Test Command")
        test_button.clicked.connect(self.send_test_command)

        # Add to layout
        test_layout.addRow("Desired Position:", self.desired_position_input)
        test_layout.addRow("Desired Velocity:", self.desired_velocity_input)
        test_layout.addWidget(test_button)

        layout.addLayout(test_layout)
        widget.setLayout(layout)
        return widget

    def create_motor_settings_tab(self):
        widget = QWidget()
        layout = QFormLayout()

        # Motor Settings fields
        self.pole_pairs = QSpinBox()
        self.pole_pairs.setRange(1, 100)
        self.pole_pairs.setValue(21)

        self.max_speed = QDoubleSpinBox()
        self.max_speed.setRange(0, 10000)
        self.max_speed.setValue(5000.0)
        self.max_speed.setSuffix(" RPM")

        layout.addRow("Pole Pairs:", self.pole_pairs)
        layout.addRow("Max Speed (RPM):", self.max_speed)

        widget.setLayout(layout)
        return widget

    def create_chart_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()

        layout.addWidget(self.canvas)
        widget.setLayout(layout)
        return widget

    def update_chart(self):
        # Update chart data here (for example, you can add real-time data from serial communication)
        self.time_data.append(len(self.time_data))  # Time axis (e.g., seconds)
        self.position_data.append(self.desired_position_input.value())  # Example: use desired position

        self.ax.clear()
        self.ax.plot(self.time_data, self.position_data, label='Position')
        self.ax.legend()

        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position/Velocity')
        self.ax.set_title('System Response')

        self.canvas.draw()

    def send_test_command(self):
        # Read the values from the input fields
        desired_position = self.desired_position_input.value()
        desired_velocity = self.desired_velocity_input.value()

        # Send the values via serial to ESP32
        if self.ser:
            command = f"{desired_position},{desired_velocity}\n"
            self.ser.write(command.encode())
            print(f"Sending command: {command}")

    def find_esp32_port(self):
        """
        Finds the ESP32's serial port by looking for known identifiers.
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "USB" in port.description or "CP210" in port.description:
                return port.device
        return None

    def connect_to_esp32(self):
        # Try to find the ESP32 port
        esp32_port = self.find_esp32_port()
        if esp32_port:
            self.ser = serial.Serial(esp32_port, 115200)  # Adjust baud rate if necessary
            print(f"Connected to ESP32 on {esp32_port}")
        else:
            print("ESP32 not found")

# Running the Application
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.connect_to_esp32()  # Automatically connect to ESP32 when starting the app
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
