from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QFrame


class SettingsDialog(QDialog):
    def __init__(self, parent, ros_bridge=None, use_ros=False, log_callback=None):
        super().__init__(parent)
        self.ros_bridge = ros_bridge
        self.use_ros = use_ros
        self.log_callback = log_callback
        self._setup_ui()
    
    def _setup_ui(self):
        self.setWindowTitle("Gazebo Environment Settings")
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground, False)
        self.setFixedSize(550, 380)
        
        # Enable dragging
        def mousePressEvent(event):
            if event.button() == Qt.LeftButton:
                self.drag_position = event.globalPos() - self.frameGeometry().topLeft()
                event.accept()
        
        def mouseMoveEvent(event):
            if event.buttons() == Qt.LeftButton and hasattr(self, 'drag_position'):
                self.move(event.globalPos() - self.drag_position)
                event.accept()
        
        self.mousePressEvent = mousePressEvent
        self.mouseMoveEvent = mouseMoveEvent
        
        self.setStyleSheet("""
            QDialog { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2d2d2d, stop:1 #1a1a1a);
                border: 2px solid #17a2b8;
                border-radius: 10px;
            }
            QLabel { color: #fff; font-size: 9pt; }
            QLineEdit { 
                background-color: #1a1a1a; 
                color: #17a2b8; 
                border: 2px solid #17a2b8; 
                border-radius: 4px; 
                padding: 6px;
                font-size: 9pt;
                font-weight: bold;
            }
            QLineEdit:focus {
                border: 2px solid #28a745;
                background-color: #0d0d0d;
            }
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #17a2b8, stop:1 #138496);
                color: white;
                border: 2px solid #0f6674;
                border-radius: 4px;
                padding: 6px 10px;
                font-weight: bold;
                font-size: 8pt;
            }
            QPushButton:hover { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #28a745, stop:1 #218838);
                border: 2px solid #1e7e34;
            }
            QPushButton:pressed { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0f6674, stop:1 #0a4f5c);
                padding-top: 8px;
            }
        """)
        
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)
        
        title = QLabel("⚙ GAZEBO WEATHER SETTINGS")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel { 
                font-size: 12pt; 
                font-weight: bold; 
                color: #17a2b8;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #1a1a1a, stop:0.5 #2a2a2a, stop:1 #1a1a1a);
                padding: 10px;
                border-radius: 6px;
                border: 2px solid #17a2b8;
            }
        """)
        layout.addWidget(title)
        
        layout.addWidget(self._create_param_row("🌡", "Temperature (°C)", "25.5", "temperature", "°C"))
        layout.addWidget(self._create_param_row("💧", "Humidity (%)", "65", "humidity", "%"))
        layout.addWidget(self._create_param_row("🌪", "Pressure (hPa)", "1013", "pressure", " hPa"))
        layout.addWidget(self._create_param_row("🌬", "Wind Speed (km/h)", "12.5", "wind_speed", " km/h"))
        layout.addWidget(self._create_param_row("☔", "Rain Rate (mm/h)", "5.0", "rain_rate", " mm/h"))
        
        layout.addStretch()
        
        close_btn = QPushButton("✖ Close")
        close_btn.setFixedWidth(100)
        close_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #6c757d, stop:1 #5a6268);
                color: white;
                border: 2px solid #495057;
                border-radius: 6px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 9pt;
            }
            QPushButton:hover { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #dc3545, stop:1 #c82333);
                border: 2px solid #bd2130;
            }
            QPushButton:pressed { 
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #545b62, stop:1 #3d4349);
                padding-top: 10px;
            }
        """)
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn, 0, Qt.AlignCenter)
    
    def _create_param_row(self, icon, label_text, placeholder, param_name, unit=""):
        container = QFrame()
        container.setStyleSheet("""
            QFrame {
                background-color: #252525;
                border-radius: 6px;
                border: 1px solid #3a3a3a;
            }
        """)
        row = QHBoxLayout(container)
        row.setSpacing(8)
        row.setContentsMargins(10, 6, 10, 6)
        
        label = QLabel(f"{icon} {label_text}")
        label.setStyleSheet("QLabel { font-weight: bold; color: #17a2b8; background: transparent; border: none; font-size: 9pt; }")
        
        input_field = QLineEdit()
        input_field.setPlaceholderText(placeholder)
        input_field.setFixedWidth(80)
        
        btn = QPushButton("✓ Apply")
        btn.setFixedWidth(70)
        
        def apply():
            try:
                if input_field.text():
                    if self.use_ros and self.ros_bridge:
                        result = self.ros_bridge.set_weather_param(param_name, float(input_field.text()))
                        success, message = (result if isinstance(result, tuple) else (bool(result), ""))
                        if success:
                            status_msg = message or f"{label_text} set to {input_field.text()}{unit}"
                            if self.log_callback:
                                self.log_callback(status_msg)
                            input_field.setStyleSheet(input_field.styleSheet() + "border: 2px solid #28a745;")
                            QTimer.singleShot(1000, lambda: input_field.setStyleSheet(""))
                        else:
                            if self.log_callback:
                                self.log_callback(f"Failed to set {label_text}: {message}")
                            input_field.setStyleSheet(input_field.styleSheet() + "border: 2px solid #dc3545;")
                            QTimer.singleShot(1000, lambda: input_field.setStyleSheet(""))
                    else:
                        if self.log_callback:
                            self.log_callback("No ROS connection")
            except ValueError:
                if self.log_callback:
                    self.log_callback(f"Error: Invalid value")
                input_field.setStyleSheet(input_field.styleSheet() + "border: 2px solid #dc3545;")
                QTimer.singleShot(1000, lambda: input_field.setStyleSheet(""))
        
        btn.clicked.connect(apply)
        row.addWidget(label)
        row.addStretch()
        row.addWidget(input_field)
        row.addWidget(btn)
        return container
