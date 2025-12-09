import sys
import os
from datetime import datetime
from pathlib import Path

# Disable Python bytecode generation
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import rospy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QDockWidget

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
        self.resize(1350, 650)
        self.setStyleSheet("QMainWindow { background-color: transparent; }")
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
        
        # Minimal central widget (hidden)
        central = QWidget()
        central.setMaximumSize(1, 1)
        self.setCentralWidget(central)
        self.setDockNestingEnabled(True)
        
        # Hide main window
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setFixedSize(1, 1)
        self.move(-10000, -10000)  # Move off-screen

        self.hard_widget = HardMMIWidget()
        self.lcd_display = LCDDisplay()
        
        # Create dock widgets
        self._create_hardware_dock()
        self._create_display_dock()
        
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

    def _create_hardware_dock(self):
        from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QGridLayout
        
        dock = QDockWidget("Hardware Control Panel", self)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        
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
        
        dock.setWidget(left_panel)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)
        dock.setFloating(True)
        dock.setWindowFlags(Qt.Window | Qt.CustomizeWindowHint | Qt.WindowTitleHint | Qt.WindowCloseButtonHint | Qt.WindowMinMaxButtonsHint)
        dock.setGeometry(50, 50, 520, 650)
        dock.hide()  # Start hidden
        self.hardware_dock = dock

    def _create_display_dock(self):
        from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QPushButton, QLabel
        from PyQt5.QtGui import QCursor
        
        dock = QDockWidget("Display Panel", self)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea | Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        
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
        
        dock.setWidget(right_panel)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        dock.setFloating(True)
        dock.setWindowFlags(Qt.Window | Qt.CustomizeWindowHint | Qt.WindowTitleHint | Qt.WindowCloseButtonHint | Qt.WindowMinMaxButtonsHint)
        dock.setGeometry(600, 50, 840, 650)
        dock.hide()  # Start hidden
        self.display_dock = dock

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
    import signal
    from PyQt5.QtWidgets import QSplashScreen
    from PyQt5.QtGui import QPixmap, QPainter, QColor, QFont
    from PyQt5.QtCore import Qt, QTimer
    import math
    
    config_path = Path(__file__).parent / "config" / "config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)
    
    use_ros = config.get('ros', {}).get('enabled', False)
    
    app = QApplication(sys.argv)
    
    # Set application icon
    icon_path = Path(__file__).parent / "assets" / "icon.png"
    if icon_path.exists():
        from PyQt5.QtGui import QIcon
        app.setWindowIcon(QIcon(str(icon_path)))
    
    # Professional animated splash
    class AnimatedSplash(QSplashScreen):
        def __init__(self):
            pix = QPixmap(400, 200)
            pix.fill(QColor("#0d1117"))
            super().__init__(pix, Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
            self.angle = 0
            self.progress = 0
            self.timer = QTimer()
            self.timer.timeout.connect(self.rotate)
            self.timer.start(50)
        
        def rotate(self):
            self.angle = (self.angle + 12) % 360
            self.repaint()
        
        def set_progress(self, value):
            self.progress = value
            self.repaint()
        
        def drawContents(self, painter):
            painter.setRenderHint(QPainter.Antialiasing)
            
            # Border
            painter.setPen(QColor("#17a2b8"))
            painter.drawRect(1, 1, 398, 198)
            
            # Logo (if exists)
            logo_path = Path(__file__).parent / "assets" / "icon.png"
            if logo_path.exists():
                from PyQt5.QtGui import QPixmap
                logo = QPixmap(str(logo_path))
                if not logo.isNull():
                    scaled_logo = logo.scaled(48, 48, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    painter.drawPixmap(20, 20, scaled_logo)
            
            # Title
            painter.setFont(QFont("Arial", 16, QFont.Bold))
            painter.setPen(QColor("#17a2b8"))
            painter.drawText(80, 50, "DOCK STATION")
            
            painter.setFont(QFont("Arial", 10))
            painter.setPen(QColor("#8b949e"))
            painter.drawText(20, 80, "Control Panel Interface")
            
            # Spinner
            for i in range(12):
                angle = math.radians(self.angle + i * 30)
                opacity = 1.0 - (i * 0.08)
                color = QColor("#17a2b8")
                color.setAlphaF(opacity)
                painter.setBrush(color)
                painter.setPen(Qt.NoPen)
                cx, cy, r = 340, 60, 20
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                painter.drawEllipse(int(x-3), int(y-3), 6, 6)
            
            # Progress bar
            painter.setPen(QColor("#30363d"))
            painter.setBrush(QColor("#0d1117"))
            painter.drawRect(20, 120, 360, 8)
            
            if self.progress > 0:
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor("#17a2b8"))
                painter.drawRect(20, 120, int(360 * self.progress / 100), 8)
            
            # Message
            painter.setFont(QFont("Arial", 9))
            painter.setPen(QColor("#8b949e"))
            super().drawContents(painter)
    
    import time
    
    splash = AnimatedSplash()
    splash.show()
    app.processEvents()
    time.sleep(0.3)
    
    splash.set_progress(10)
    splash.showMessage("  Initializing...", Qt.AlignBottom | Qt.AlignLeft, QColor("#8b949e"))
    app.processEvents()
    time.sleep(0.4)
    
    if use_ros:
        splash.set_progress(25)
        splash.showMessage("  Connecting to ROS...", Qt.AlignBottom | Qt.AlignLeft, QColor("#8b949e"))
        app.processEvents()
        time.sleep(0.3)
        rospy.init_node('dock_station_ui', anonymous=True)
        
        # Ctrl+C sinyalini yakala ve Qt'yi kapat
        def sigint_handler(*args):
            rospy.signal_shutdown('SIGINT')
            QApplication.quit()
            sys.exit(0)
        signal.signal(signal.SIGINT, sigint_handler)
        
        # ROS shutdown olduğunda Qt uygulamasını kapat
        rospy.on_shutdown(lambda: QApplication.quit())
        
        splash.set_progress(50)
        splash.showMessage("  Loading services...", Qt.AlignBottom | Qt.AlignLeft, QColor("#8b949e"))
        app.processEvents()
        time.sleep(0.4)
    
    splash.set_progress(75)
    splash.showMessage("  Building interface...", Qt.AlignBottom | Qt.AlignLeft, QColor("#8b949e"))
    app.processEvents()
    time.sleep(0.3)
    
    panel = TelemetryPanel(use_ros=use_ros)
    
    splash.set_progress(100)
    splash.showMessage("  Ready!", Qt.AlignBottom | Qt.AlignLeft, QColor("#17a2b8"))
    app.processEvents()
    time.sleep(0.4)
    
    def show_panels():
        # Wait a bit more for initial ROS messages if using ROS
        if use_ros:
            time.sleep(0.8)
            app.processEvents()
        
        panel.hardware_dock.show()
        panel.display_dock.show()
        panel.show()
        splash.finish(panel)
        
        # Timer to allow Ctrl+C to work
        if use_ros:
            timer = QTimer()
            timer.timeout.connect(lambda: None)
            timer.start(100)
    
    QTimer.singleShot(100, show_panels)
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
