import sys
import threading
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QPushButton, QLineEdit, 
                               QGridLayout, QGroupBox, QComboBox, QMessageBox, QDoubleSpinBox)
from PySide6.QtCore import QTimer, Qt, Signal, QPointF, QRectF
from PySide6.QtGui import QFont, QColor, QPalette, QPainter, QPen, QBrush

import DobotDllType as dType

class WorkspaceMap(QWidget):
    target_clicked = Signal(float, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        
        self.max_reach = 320.0
        self.min_reach = 150.0
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Background
        painter.fillRect(self.rect(), QColor("#11111b"))
        
        w = self.width()
        h = self.height()
        
        # Origin at bottom center
        origin_x = w / 2
        origin_y = h - 30
        
        # Scale
        scale = min(w / 2 - 20, h - 50) / 350.0
        if scale <= 0: return
        
        def to_screen(rx, ry):
            # Dobot X is forward, Y is left. So X -> Up, Y -> Left
            return origin_x - ry * scale, origin_y - rx * scale

        # Grid
        painter.setPen(QPen(QColor("#313244"), 1, Qt.DashLine))
        for x in range(0, 351, 50):
            sx, sy = to_screen(x, 0)
            painter.drawLine(0, sy, w, sy)
            painter.drawText(w - 30, sy - 5, f"{x}")
        for y in range(-350, 351, 50):
            sx, sy = to_screen(0, y)
            painter.drawLine(sx, 0, sx, h)
            if y != 0:
                painter.drawText(sx + 5, 15, f"{y}")
            
        # Axes
        painter.setPen(QPen(QColor("#89b4fa"), 2))
        sx, sy = to_screen(0, 0)
        ex, ey = to_screen(350, 0)
        painter.drawLine(sx, sy, ex, ey) # X axis
        painter.drawText(ex + 5, ey, "X")
        
        ex, ey = to_screen(0, -350)
        painter.drawLine(sx, sy, ex, ey) # -Y axis
        painter.drawText(ex, ey - 5, "-Y (Right)")
        
        ex, ey = to_screen(0, 350)
        painter.drawLine(sx, sy, ex, ey) # +Y axis
        painter.drawText(ex - 60, ey - 5, "+Y (Left)")
        
        # Work area arcs
        painter.setPen(QPen(QColor("#a6e3a1"), 2, Qt.DotLine))
        rect_max = QRectF(origin_x - self.max_reach * scale, origin_y - self.max_reach * scale, 
                          self.max_reach * 2 * scale, self.max_reach * 2 * scale)
        painter.drawArc(rect_max, 0 * 16, 180 * 16)
        
        rect_min = QRectF(origin_x - self.min_reach * scale, origin_y - self.min_reach * scale, 
                          self.min_reach * 2 * scale, self.min_reach * 2 * scale)
        painter.drawArc(rect_min, 0 * 16, 180 * 16)
        
        # Target pos
        tx, ty = to_screen(self.target_x, self.target_y)
        painter.setPen(QPen(QColor("#f38ba8"), 2))
        painter.drawLine(tx - 6, ty, tx + 6, ty)
        painter.drawLine(tx, ty - 6, tx, ty + 6)
        
        # Current pos
        cx, cy = to_screen(self.current_x, self.current_y)
        painter.setPen(QPen(QColor("#a6e3a1"), 3))
        painter.setBrush(QBrush(QColor("#a6e3a1")))
        painter.drawEllipse(QPointF(cx, cy), 5, 5)
        
        # Arm line
        painter.setPen(QPen(QColor("#9399b2"), 4, Qt.SolidLine))
        painter.drawLine(origin_x, origin_y, cx, cy)
        
        # Current coordinates text
        painter.setPen(QPen(QColor("#cdd6f4"), 1))
        painter.drawText(10, h - 10, f"Pos: ({self.current_x:.1f}, {self.current_y:.1f}) Target: ({self.target_x:.1f}, {self.target_y:.1f})")

    def mousePressEvent(self, event):
        w = self.width()
        h = self.height()
        origin_x = w / 2
        origin_y = h - 30
        scale = min(w / 2 - 20, h - 50) / 350.0
        if scale <= 0: return
        
        rx = (origin_y - event.position().y()) / scale
        ry = (origin_x - event.position().x()) / scale
        
        # Simple bounding
        if rx < -50: rx = -50
        if rx > 400: rx = 400
        if ry < -400: ry = -400
        if ry > 400: ry = 400
        
        self.target_x = rx
        self.target_y = ry
        self.update()
        self.target_clicked.emit(self.target_x, self.target_y)

class DobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dobot Magician Control Panel")
        self.setMinimumSize(900, 500)
        
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
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(20)
        
        # --- Left Panel: Workspace Map ---
        left_layout = QVBoxLayout()
        map_group = QGroupBox("2D Workspace Map (Top View)")
        map_layout = QVBoxLayout()
        self.workspace_map = WorkspaceMap()
        self.workspace_map.target_clicked.connect(self.on_map_clicked)
        map_layout.addWidget(self.workspace_map)
        
        lbl_info = QLabel("Click on the map to set X/Y target coordinates.")
        lbl_info.setStyleSheet("color: #89b4fa; font-size: 12px;")
        lbl_info.setAlignment(Qt.AlignCenter)
        map_layout.addWidget(lbl_info)
        map_group.setLayout(map_layout)
        left_layout.addWidget(map_group)
        main_layout.addLayout(left_layout, 1)
        
        # --- Right Panel: Controls ---
        right_layout = QVBoxLayout()
        
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
        right_layout.addWidget(conn_group)
        
        # --- Axis Control Panel ---
        axis_group = QGroupBox("Axis Control & Targets")
        axis_layout = QGridLayout()
        
        headers = ["Axis", "Current", "Target", "Jog -", "Jog +"]
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setFont(QFont("Arial", 10, QFont.Bold))
            lbl.setAlignment(Qt.AlignCenter)
            axis_layout.addWidget(lbl, 0, col)
            
        self.axes = ["X", "Y", "Z", "R", "L"]
        self.ui_elements = {}
        
        for row, axis in enumerate(self.axes, start=1):
            lbl_axis = QLabel(axis)
            lbl_axis.setAlignment(Qt.AlignCenter)
            lbl_axis.setFont(QFont("Arial", 12, QFont.Bold))
            
            lbl_current = QLabel("0.00")
            lbl_current.setAlignment(Qt.AlignCenter)
            lbl_current.setStyleSheet("color: #55ff55; font-weight: bold; font-size: 14px;")
            
            spin_target = QDoubleSpinBox()
            spin_target.setRange(-2000, 2000)
            spin_target.setDecimals(2)
            spin_target.setValue(0.0)
            if axis == "X":
                spin_target.valueChanged.connect(self.on_target_x_changed)
            elif axis == "Y":
                spin_target.valueChanged.connect(self.on_target_y_changed)
            
            btn_minus = QPushButton(f"-")
            btn_minus.setFixedWidth(50)
            btn_minus.clicked.connect(lambda checked, a=axis, d=-1: self.jog(a, d))
            
            btn_plus = QPushButton(f"+")
            btn_plus.setFixedWidth(50)
            btn_plus.clicked.connect(lambda checked, a=axis, d=1: self.jog(a, d))
            
            axis_layout.addWidget(lbl_axis, row, 0)
            axis_layout.addWidget(lbl_current, row, 1)
            axis_layout.addWidget(spin_target, row, 2)
            axis_layout.addWidget(btn_minus, row, 3, Qt.AlignCenter)
            axis_layout.addWidget(btn_plus, row, 4, Qt.AlignCenter)
            
            self.ui_elements[axis] = {
                "current": lbl_current,
                "target": spin_target,
                "btn_minus": btn_minus,
                "btn_plus": btn_plus
            }
            
        axis_group.setLayout(axis_layout)
        right_layout.addWidget(axis_group)
        
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
        
        self.move_coords_btn = QPushButton("Move to Targets")
        self.move_coords_btn.setMinimumHeight(50)
        self.move_coords_btn.setStyleSheet("background-color: #2ea043; color: white; font-weight: bold; font-size: 14px;")
        self.move_coords_btn.clicked.connect(self.move_to_targets)

        self.home_btn = QPushButton("HOME")
        self.home_btn.setMinimumHeight(50)
        self.home_btn.setStyleSheet("background-color: #0078d7; color: white; font-weight: bold; font-size: 14px;")
        self.home_btn.clicked.connect(self.go_home)
        
        bottom_layout.addWidget(step_group)
        bottom_layout.addWidget(self.move_coords_btn)
        bottom_layout.addWidget(self.home_btn)
        
        right_layout.addLayout(bottom_layout)
        main_layout.addLayout(right_layout)
        
        self.set_controls_enabled(False)

    def on_map_clicked(self, x, y):
        self.ui_elements["X"]["target"].setValue(x)
        self.ui_elements["Y"]["target"].setValue(y)

    def on_target_x_changed(self, val):
        self.workspace_map.target_x = val
        self.workspace_map.update()

    def on_target_y_changed(self, val):
        self.workspace_map.target_y = val
        self.workspace_map.update()
        
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
            self.ui_elements[axis]["target"].setEnabled(enabled)
        self.home_btn.setEnabled(enabled)
        self.move_coords_btn.setEnabled(enabled)
        
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
                
                dType.dSleep(500)
                # Clear queue and configure
                dType.SetQueuedCmdClear(self.api)
                dType.SetWAITCmd(self.api, 100, isQueued=1)
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
                
            self.workspace_map.current_x = pose[0]
            self.workspace_map.current_y = pose[1]
            self.workspace_map.update()
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
        
        # Send move command (using PTPMOVLXYZMode for linear movements)
        # For rail movements, we use SetPTPWithLCmd
        dType.SetPTPWithLCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, target_x, target_y, target_z, target_r, target_l, isQueued=1)
        
    def move_to_targets(self):
        if not self.connected: return
        
        target_x = self.ui_elements["X"]["target"].value()
        target_y = self.ui_elements["Y"]["target"].value()
        target_z = self.ui_elements["Z"]["target"].value()
        target_r = self.ui_elements["R"]["target"].value()
        target_l = self.ui_elements["L"]["target"].value()
        
        # Send move command (using PTPMOVLXYZMode for linear movements)
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
