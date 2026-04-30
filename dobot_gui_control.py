import sys
import threading
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QPushButton, QLineEdit, 
                               QGridLayout, QGroupBox, QComboBox, QMessageBox, QDoubleSpinBox)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont, QColor, QPalette

import DobotDllType as dType

class DobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dobot Magician Control Panel")
        self.setMinimumSize(600, 500)
        
        self.api = dType.load()
        self.connected = False
        
        self.current_pose = {"X": 0.0, "Y": 0.0, "Z": 0.0, "R": 0.0, "L": 0.0}
        
        # UI Setup
        self.setup_ui()
        self.apply_dark_theme()
        
        # Timer for polling position
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.update_pose)
        
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(20)
        
        # --- Connection Panel ---
        conn_group = QGroupBox("Connection")
        conn_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        # Default to COM3 based on user preferences
        self.port_combo.addItems([f"COM{i}" for i in range(1, 20)])
        self.port_combo.setCurrentText("COM3")
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: #ff5555; font-weight: bold;")
        
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.status_label)
        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)
        main_layout.addWidget(conn_group)
        
        # --- Axis Control Panel ---
        axis_group = QGroupBox("Axis Control & Limits")
        axis_layout = QGridLayout()
        
        headers = ["Axis", "Current", "Min Limit", "Max Limit", "Jog -", "Jog +"]
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setFont(QFont("Arial", 10, QFont.Bold))
            lbl.setAlignment(Qt.AlignCenter)
            axis_layout.addWidget(lbl, 0, col)
            
        self.axes = ["X", "Y", "Z", "R", "L"]
        self.ui_elements = {}
        
        # Default Limits (mm/degrees)
        default_limits = {
            "X": (100.0, 350.0),
            "Y": (-300.0, 300.0),
            "Z": (-50.0, 200.0),
            "R": (-180.0, 180.0),
            "L": (0.0, 1000.0)
        }
        
        for row, axis in enumerate(self.axes, start=1):
            lbl_axis = QLabel(axis)
            lbl_axis.setAlignment(Qt.AlignCenter)
            lbl_axis.setFont(QFont("Arial", 12, QFont.Bold))
            
            lbl_current = QLabel("0.00")
            lbl_current.setAlignment(Qt.AlignCenter)
            lbl_current.setStyleSheet("color: #55ff55; font-weight: bold; font-size: 14px;")
            
            spin_min = QDoubleSpinBox()
            spin_min.setRange(-2000, 2000)
            spin_min.setValue(default_limits[axis][0])
            
            spin_max = QDoubleSpinBox()
            spin_max.setRange(-2000, 2000)
            spin_max.setValue(default_limits[axis][1])
            
            btn_minus = QPushButton(f"-")
            btn_minus.setFixedWidth(50)
            btn_minus.clicked.connect(lambda checked, a=axis, d=-1: self.jog(a, d))
            
            btn_plus = QPushButton(f"+")
            btn_plus.setFixedWidth(50)
            btn_plus.clicked.connect(lambda checked, a=axis, d=1: self.jog(a, d))
            
            axis_layout.addWidget(lbl_axis, row, 0)
            axis_layout.addWidget(lbl_current, row, 1)
            axis_layout.addWidget(spin_min, row, 2)
            axis_layout.addWidget(spin_max, row, 3)
            axis_layout.addWidget(btn_minus, row, 4, Qt.AlignCenter)
            axis_layout.addWidget(btn_plus, row, 5, Qt.AlignCenter)
            
            self.ui_elements[axis] = {
                "current": lbl_current,
                "min": spin_min,
                "max": spin_max,
                "btn_minus": btn_minus,
                "btn_plus": btn_plus
            }
            
        axis_group.setLayout(axis_layout)
        main_layout.addWidget(axis_group)
        
        # --- Settings and Home ---
        bottom_layout = QHBoxLayout()
        
        step_group = QGroupBox("Jog Step")
        step_layout = QHBoxLayout()
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.1, 100.0)
        self.step_spin.setValue(10.0)
        step_layout.addWidget(QLabel("Step Size:"))
        step_layout.addWidget(self.step_spin)
        step_group.setLayout(step_layout)
        
        self.home_btn = QPushButton("HOME")
        self.home_btn.setMinimumHeight(50)
        self.home_btn.setStyleSheet("background-color: #0078d7; color: white; font-weight: bold; font-size: 14px;")
        self.home_btn.clicked.connect(self.go_home)
        
        bottom_layout.addWidget(step_group)
        bottom_layout.addWidget(self.home_btn)
        
        main_layout.addLayout(bottom_layout)
        
        self.set_controls_enabled(False)
        
    def apply_dark_theme(self):
        # Dark theme stylesheet for a modern rich aesthetic
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2e;
            }
            QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QGroupBox {
                border: 2px solid #313244;
                border-radius: 8px;
                margin-top: 1.5ex;
                font-weight: bold;
                color: #89b4fa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 10px;
            }
            QPushButton {
                background-color: #45475a;
                border: none;
                border-radius: 6px;
                padding: 8px 12px;
                color: #cdd6f4;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #585b70;
            }
            QPushButton:pressed {
                background-color: #313244;
            }
            QPushButton:disabled {
                background-color: #181825;
                color: #45475a;
            }
            QDoubleSpinBox, QComboBox {
                background-color: #11111b;
                border: 1px solid #313244;
                border-radius: 4px;
                padding: 4px;
                color: #cdd6f4;
            }
            QDoubleSpinBox:focus, QComboBox:focus {
                border: 1px solid #89b4fa;
            }
        """)
        
    def set_controls_enabled(self, enabled):
        for axis in self.axes:
            self.ui_elements[axis]["btn_minus"].setEnabled(enabled)
            self.ui_elements[axis]["btn_plus"].setEnabled(enabled)
        self.home_btn.setEnabled(enabled)
        
    def toggle_connection(self):
        if not self.connected:
            port = self.port_combo.currentText()
            state = dType.ConnectDobot(self.api, port, 115200)[0]
            
            if state == dType.DobotConnect.DobotConnect_NoError:
                self.connected = True
                self.connect_btn.setText("Disconnect")
                self.status_label.setText("Status: Connected")
                self.status_label.setStyleSheet("color: #55ff55; font-weight: bold;")
                self.set_controls_enabled(True)
                
                # Clear queue and configure
                dType.SetQueuedCmdClear(self.api)
                # Enable rail
                dType.SetDeviceWithL(self.api, True, version=1)
                
                dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
                dType.SetPTPCommonParams(self.api, 100, 100, isQueued=1)
                dType.SetPTPLParams(self.api, 99, 99, isQueued=1)
                dType.SetQueuedCmdStartExec(self.api)
                
                self.poll_timer.start(500)
            else:
                QMessageBox.critical(self, "Connection Error", f"Failed to connect to {port}. Is the robot powered on and not used by another program?")
        else:
            self.poll_timer.stop()
            dType.SetQueuedCmdStopExec(self.api)
            dType.DisconnectDobot(self.api)
            self.connected = False
            self.connect_btn.setText("Connect")
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("color: #ff5555; font-weight: bold;")
            self.set_controls_enabled(False)
            
    def update_pose(self):
        if not self.connected:
            return
            
        try:
            pose = dType.GetPose(self.api)
            rail_pose = dType.GetPoseL(self.api)
            
            self.current_pose["X"] = pose[0]
            self.current_pose["Y"] = pose[1]
            self.current_pose["Z"] = pose[2]
            self.current_pose["R"] = pose[3]
            self.current_pose["L"] = rail_pose[0]
            
            for axis in self.axes:
                self.ui_elements[axis]["current"].setText(f"{self.current_pose[axis]:.2f}")
        except Exception as e:
            print(f"Error polling pose: {e}")
            
    def jog(self, axis, direction):
        if not self.connected: return
        
        step = self.step_spin.value()
        delta = step * direction
        
        # Calculate target position based on current target values
        target_x = self.current_pose["X"]
        target_y = self.current_pose["Y"]
        target_z = self.current_pose["Z"]
        target_r = self.current_pose["R"]
        target_l = self.current_pose["L"]
        
        if axis == "X": target_x += delta
        elif axis == "Y": target_y += delta
        elif axis == "Z": target_z += delta
        elif axis == "R": target_r += delta
        elif axis == "L": target_l += delta
        
        # Validate limits
        limit_min = self.ui_elements[axis]["min"].value()
        limit_max = self.ui_elements[axis]["max"].value()
        target_val = locals()[f"target_{axis.lower()}"]
        
        if target_val < limit_min or target_val > limit_max:
            QMessageBox.warning(self, "Limit Exceeded", f"Cannot move {axis} to {target_val:.2f}.\nLimit is [{limit_min}, {limit_max}].")
            return
            
        # Send move command (using PTPMOVLXYZMode for linear movements)
        # For rail movements, we use SetPTPWithLCmd
        dType.SetPTPWithLCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, target_x, target_y, target_z, target_r, target_l, isQueued=1)
        
    def go_home(self):
        if not self.connected: return
        
        reply = QMessageBox.question(self, 'Confirm Home', 
                                     'Are you sure you want to return the robot to the Home position?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Set HOME parameters and trigger home command
            dType.SetHOMEParams(self.api, 200, 200, 200, 200, isQueued=1)
            dType.SetHOMECmd(self.api, temp=0, isQueued=1)

    def closeEvent(self, event):
        if self.connected:
            dType.DisconnectDobot(self.api)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    # Global app styling
    app.setStyle("Fusion")
    window = DobotGUI()
    window.show()
    sys.exit(app.exec())
