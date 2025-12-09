import sys
import os
from datetime import datetime
from pathlib import Path

# Disable Python bytecode generation
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import rospy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget

if __name__ == "__main__":
    PACKAGE_ROOT = Path(__file__).resolve().parent
    if str(PACKAGE_ROOT.parent) not in sys.path:
        sys.path.insert(0, str(PACKAGE_ROOT.parent))
    from dock_sim_ui.core import sample_telemetry, ROSBridge
    from dock_sim_ui.ui import LCDDisplay, HardMMIWidget, SettingsDialog
else:
    from .core import sample_telemetry, ROSBridge
    from .ui import LCDDisplay, HardMMIWidget, SettingsDialog


class TelemetryPanel(QMainWindow):
    def __init__(self, use_ros=True):
        super().__init__()
        self.setWindowTitle("Dock Station Control Panel")
        self.setFixedSize(1350, 650)
        self.setStyleSheet("QMainWindow { background-color: #2d2d2d; }")
        self.use_ros = use_ros
        self.telemetry = sample_telemetry()
        self._build_ui()
        
        if use_ros:
            self.ros_bridge = ROSBridge()
            self.ros_bridge.telemetry_updated.connect(self.update_telemetry)
        else:
            self.update_telemetry(self.telemetry)
            self._start_demo_timer()

    def _build_ui(self):
        from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel
        central = QWidget()
        central.setStyleSheet("QWidget { background-color: #2d2d2d; }")
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self.hard_widget = HardMMIWidget()
        self.lcd_display = LCDDisplay()
        
        self.lcd_display.page_selected.connect(self._log_tab_change)
        self.hard_widget.estop_released.connect(lambda: self._handle_estop(False))
        self.hard_widget.estop_engaged.connect(lambda: self._handle_estop(True))
        self.hard_widget.lid_open_pressed.connect(lambda: self._dispatch_action("LID_OPEN"))
        self.hard_widget.lid_close_pressed.connect(lambda: self._dispatch_action("LID_CLOSE"))
        self.hard_widget.lid_stop_pressed.connect(lambda: self._dispatch_action("LID_STOP"))
        self.hard_widget.manual_open_pressed.connect(
            lambda: self._manual_control("MANUAL_OPEN", "OPENING")
        )
        self.hard_widget.manual_open_released.connect(
            lambda: self._manual_control("MANUAL_OPEN_RELEASED", "IDLE")
        )
        self.hard_widget.manual_close_pressed.connect(
            lambda: self._manual_control("MANUAL_CLOSE", "CLOSING")
        )
        self.hard_widget.manual_close_released.connect(
            lambda: self._manual_control("MANUAL_CLOSE_RELEASED", "IDLE")
        )
        self.hard_widget.network_test_pressed.connect(self._run_network_test)

        left_panel = QWidget()
        left_panel.setFixedWidth(520)
        left_panel.setStyleSheet("QWidget { background-color: #3a3a3a; }")
        main_left_layout = QVBoxLayout(left_panel)
        main_left_layout.setContentsMargins(25, 35, 25, 35)
        main_left_layout.setSpacing(25)
        
        led_row = QHBoxLayout()
        led_row.setSpacing(12)
        led_row.addStretch()
        led_row.addWidget(self.hard_widget.system_led_panel)
        led_row.addWidget(self.hard_widget.speaker_panel)
        led_row.addWidget(self.hard_widget.lid_led_panel)
        led_row.addStretch()
        
        button_container = QHBoxLayout()
        button_container.addStretch()
        
        button_grid = QGridLayout()
        button_grid.setSpacing(15)
        button_grid.addWidget(self.hard_widget.btn_lid_open, 0, 0)
        button_grid.addWidget(self.hard_widget.btn_lid_stop, 0, 1)
        button_grid.addWidget(self.hard_widget.btn_lid_close, 0, 2)
        button_grid.addWidget(self.hard_widget.btn_manual_open, 1, 0)
        button_grid.addWidget(self.hard_widget.btn_estop, 1, 1, Qt.AlignCenter)
        button_grid.addWidget(self.hard_widget.btn_manual_close, 1, 2)
        
        button_container.addLayout(button_grid)
        button_container.addStretch()
        
        main_left_layout.addLayout(led_row)
        main_left_layout.addLayout(button_container)
        main_left_layout.addStretch()
        
        right_panel = QWidget()
        right_panel.setStyleSheet("QWidget { background-color: #1a1a1a; }")
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(60, 30, 60, 30)
        right_layout.setSpacing(16)
        
        from PyQt5.QtWidgets import QLabel
        from PyQt5.QtGui import QCursor
        self.title = QLabel("DOCK STATION PANEL")
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setFixedWidth(720)
        self.title.setCursor(QCursor(Qt.PointingHandCursor))
        self.title.setStyleSheet("""
            QLabel {
                color: #ffffff;
                font-size: 20pt;
                font-weight: bold;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #1a1a1a, stop:0.5 #2a2a2a, stop:1 #1a1a1a);
                padding: 15px 20px;
                border-radius: 10px;
                border: 3px solid #17a2b8;
                letter-spacing: 4px;
            }
        """)
        self.title.mousePressEvent = lambda e: self._open_settings()
        right_layout.addWidget(self.title, 0, Qt.AlignCenter)
        right_layout.addWidget(self.lcd_display, 0, Qt.AlignCenter)
        
        nav_buttons = QHBoxLayout()
        nav_buttons.setSpacing(15)
        
        page_names = ["HOME", "WEATHER", "LOCATION", "SYSTEM", "CHARGER"]
        for idx, name in enumerate(page_names):
            btn = QPushButton(name)
            btn.setFixedSize(135, 50)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2d2d2d;
                    color: white;
                    border: 4px outset #555;
                    border-radius: 6px;
                    font-weight: bold;
                    font-size: 9pt;
                }
                QPushButton:pressed {
                    background-color: #3a3a3a;
                    border: 4px inset #333;
                    padding-top: 2px;
                }
            """)
            btn.clicked.connect(lambda _, i=idx: self.lcd_display.set_page(i))
            nav_buttons.addWidget(btn)
        
        right_layout.addLayout(nav_buttons)
        right_layout.addStretch()
        
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel)
        
        self.setCentralWidget(central)

    def update_telemetry(self, data: dict):
        telemetry = data.get("telemetry", {})
        lid = telemetry.get("lid", {})
        weather = telemetry.get("weather", {})
        location = telemetry.get("location", {})
        system = telemetry.get("system", {})
        charger = telemetry.get("charger", {})

        state = lid.get("state", 0)
        error_code = system.get("error", {}).get("code", 0)

        self.lcd_display.update_summary(state, lid, weather, system, charger, location)
        self.lcd_display.update_weather(weather)
        self.lcd_display.update_location(location)
        self.lcd_display.update_system(system)
        self.lcd_display.update_charger(charger)
        self.lcd_display.update_status_banner(self._status_text(state, error_code), self._status_color(state, error_code))
        self.hard_widget.update_leds(state, error_code)
        self.hard_widget.set_buzzer(error_code != 0 or state == 6 or self.hard_widget.estop_active)

    def append_log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.lcd_display.append_log(f"[{timestamp}] {message}")

    def _dispatch_action(self, action: str):
        self.append_log(f"{action} button pressed")
        
        # Speak action for OPEN and CLOSE buttons
        if action == "LID_OPEN":
            self.hard_widget.speak_message("Opening the lid.")
        elif action == "LID_CLOSE":
            self.hard_widget.speak_message("Closing the lid.")
        elif action == "LID_STOP":
            self.hard_widget.speak_message("Lid stopped.")
        
        if self.use_ros:
            success, msg = self.ros_bridge.call_action(action)
            self.append_log(f"Service: {msg}")

    def _manual_state(self, status: str):
        self.hard_widget.set_manual_status(f"Manual control: {status}")
        if status in ("OPENING", "CLOSING"):
            self.lcd_display.update_status_banner(f"MANUAL MODE ({status})", "#17a2b8")
            self.append_log(f"Manual mode: {status}")
        else:
            self.lcd_display.update_status_banner("SYSTEM READY")
    
    def _manual_control(self, action: str, status: str):
        self._manual_state(status)
        if self.use_ros and action:
            success, msg = self.ros_bridge.call_action(action)
            self.append_log(f"Manual service: {msg}")

    def _handle_estop(self, engaged: bool):
        self.append_log("E-STOP ENGAGED" if engaged else "E-STOP RELEASED")
        self.hard_widget.set_manual_status("Manual control: STOPPED" if engaged else "Manual control: IDLE")
        color = "#dc3545" if engaged else "#11324b"
        self.lcd_display.update_status_banner("EMERGENCY STOP ACTIVE" if engaged else "SYSTEM READY", color)

    def _run_network_test(self):
        self.append_log("Network test initiated")
        self.hard_widget.set_network_result("Pinging 8.8.8.8...")
        QTimer.singleShot(1200, lambda: self.hard_widget.set_network_result("Ping test OK"))

    def _status_text(self, state: int, error_code: int) -> str:
        if self.hard_widget.estop_active:
            return "EMERGENCY STOP ACTIVE"
        if error_code != 0 or state == 6:
            return "ERROR"
        if state in (1, 2):
            return "MOVING"
        return "SYSTEM OK"

    def _status_color(self, state: int, error_code: int) -> str:
        if self.hard_widget.estop_active:
            return "#dc3545"
        if error_code != 0 or state == 6:
            return "#dc3545"
        if state in (1, 2):
            return "#17a2b8"
        return "#28a745"

    def _log_tab_change(self, index: int):
        self.append_log(f"Switched to tab {index}")

    def _start_demo_timer(self):
        self._demo_timer = QTimer(self)
        self._demo_timer.timeout.connect(self._cycle_light)
        self._demo_timer.start(8000)
        
        # Mock error simulation for testing buzzer
        self._error_timer = QTimer(self)
        self._error_timer.timeout.connect(self._cycle_error)
        self._error_timer.start(15000)  # Trigger error every 15 seconds

    def _cycle_light(self):
        env = self.telemetry["telemetry"]["weather"]["env"]
        current = env.get("light", 0)
        env["light"] = 2000 if current < 3000 else 100
        self.update_telemetry(self.telemetry)
    
    def _cycle_error(self):
        """Mock error simulation to test buzzer"""
        error = self.telemetry["telemetry"]["system"]["error"]
        current_code = error.get("code", 0)
        
        if current_code == 0:
            # Trigger error
            error["code"] = 1
            error["msg"] = "Mock error for testing"
            self.append_log("Mock error triggered - buzzer should sound")
        else:
            # Clear error
            error["code"] = 0
            error["msg"] = "OK"
            self.append_log("Mock error cleared - buzzer should stop")
        
        self.update_telemetry(self.telemetry)
    
    def _open_settings(self):
        ros_bridge = self.ros_bridge if self.use_ros and hasattr(self, 'ros_bridge') else None
        dialog = SettingsDialog(self, ros_bridge, self.use_ros, self.append_log)
        dialog.show()


def main():
    import yaml
    
    config_path = Path(__file__).parent / "config" / "config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)
    
    use_ros = config.get('ros', {}).get('enabled', False)
    
    if use_ros:
        rospy.init_node('dock_station_ui', anonymous=True)
    
    app = QApplication(sys.argv)
    panel = TelemetryPanel(use_ros=use_ros)
    panel.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
