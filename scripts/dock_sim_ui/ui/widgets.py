from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


class LEDIndicator(QWidget):
    def __init__(self, label_text: str, size: int = 28):
        super().__init__()
        self.setFixedSize(140, 70)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.setAlignment(Qt.AlignHCenter)
        
        self.dot = QLabel()
        self.dot.setFixedSize(size, size)
        self.dot.setStyleSheet(f"""
            QLabel {{
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                    fx:0.3, fy:0.3, stop:0 #2a2a2a, stop:0.7 #1a1a1a, stop:1 #0a0a0a);
                border: 3px solid #0a0a0a;
                border-radius: {size//2}px;
            }}
        """)
        
        self.caption = QLabel(label_text)
        self.caption.setAlignment(Qt.AlignCenter)
        self.caption.setStyleSheet("""
            QLabel {
                font-size: 9pt;
                color: #b0b0b0;
                background-color: transparent;
                font-weight: bold;
                font-family: Arial;
                padding: 4px 8px;
            }
        """)
        
        layout.addWidget(self.dot, alignment=Qt.AlignCenter)
        layout.addWidget(self.caption, alignment=Qt.AlignCenter)

    def set_color(self, color: str):
        size = self.dot.width()
        if color == "gray" or color == "#333":
            self.dot.setStyleSheet(f"""
                QLabel {{
                    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                        fx:0.3, fy:0.3, stop:0 #2a2a2a, stop:0.7 #1a1a1a, stop:1 #0a0a0a);
                    border: 3px solid #0a0a0a;
                    border-radius: {size//2}px;
                }}
            """)
            self.dot.setGraphicsEffect(None)
        else:
            self.dot.setStyleSheet(f"""
                QLabel {{
                    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                        fx:0.3, fy:0.3, stop:0 {color}, stop:0.6 {color}, stop:1 #0a0a0a);
                    border: 3px solid {color};
                    border-radius: {size//2}px;
                }}
            """)
            self.dot.setGraphicsEffect(self._create_glow(color))
    
    def _create_glow(self, color: str):
        from PyQt5.QtWidgets import QGraphicsDropShadowEffect
        from PyQt5.QtGui import QColor
        glow = QGraphicsDropShadowEffect()
        glow.setBlurRadius(35)
        glow.setColor(QColor(color))
        glow.setOffset(0, 0)
        return glow


class HardMMIWidget(QWidget):
    estop_engaged = pyqtSignal()
    estop_released = pyqtSignal()
    lid_open_pressed = pyqtSignal()
    lid_open_released = pyqtSignal()
    lid_close_pressed = pyqtSignal()
    lid_close_released = pyqtSignal()
    lid_stop_pressed = pyqtSignal()
    lid_stop_released = pyqtSignal()
    manual_open_pressed = pyqtSignal()
    manual_open_released = pyqtSignal()
    manual_close_pressed = pyqtSignal()
    manual_close_released = pyqtSignal()
    network_test_pressed = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.estop_active = False
        self.buzzer_active = False
        self.buzzer_timer = None
        self.tts_engine = None
        self._create_buttons()
        self._create_panels()
        self._init_tts()
    
    def _create_buttons(self):
        self.btn_estop = self._make_estop_button()

        self.btn_lid_open = self._make_standard_button("OPEN", "#0d5a27", "#1a8a3f")
        self.btn_lid_open.pressed.connect(self.lid_open_pressed.emit)
        self.btn_lid_open.released.connect(self.lid_open_released.emit)

        self.btn_lid_close = self._make_standard_button("CLOSE", "#163679", "#2557b8")
        self.btn_lid_close.pressed.connect(self.lid_close_pressed.emit)
        self.btn_lid_close.released.connect(self.lid_close_released.emit)

        self.btn_lid_stop = self._make_standard_button("STOP", "#8b6914", "#ffa500")
        self.btn_lid_stop.pressed.connect(self.lid_stop_pressed.emit)
        self.btn_lid_stop.released.connect(self.lid_stop_released.emit)

        self.btn_manual_open = self._make_standard_button("▲ OPEN", "#2d2d2d", "#555555")
        self.btn_manual_open.pressed.connect(self.manual_open_pressed.emit)
        self.btn_manual_open.released.connect(self.manual_open_released.emit)

        self.btn_manual_close = self._make_standard_button("▼ CLOSE", "#2d2d2d", "#555555")
        self.btn_manual_close.pressed.connect(self.manual_close_pressed.emit)
        self.btn_manual_close.released.connect(self.manual_close_released.emit)
    
    def _create_panels(self):
        self.speaker_panel = self._create_speaker_panel()
        self._create_status_panel()
        
        self.lid_led_panel = QFrame()
        self.lid_led_panel.setFixedSize(140, 380)
        self.lid_led_panel.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2a2a2a, stop:0.5 #1a1a1a, stop:1 #0f0f0f);
                border: 2px solid #444;
                border-radius: 12px;
            }
        """)
        lid_layout = QVBoxLayout(self.lid_led_panel)
        lid_layout.setSpacing(20)
        lid_layout.setContentsMargins(0, 25, 0, 25)
        lid_layout.setAlignment(Qt.AlignCenter)
        
        lid_label = QLabel("LID")
        lid_label.setAlignment(Qt.AlignCenter)
        lid_label.setFixedWidth(130)
        lid_label.setStyleSheet("""
            QLabel {
                color: #e0e0e0;
                font-size: 11pt;
                font-weight: bold;
                background-color: #1a1a1a;
                padding: 8px;
                border-radius: 6px;
                border: 1px solid #555;
            }
        """)
        lid_layout.addWidget(lid_label, 0, Qt.AlignCenter)
        
        self.open_led = LEDIndicator("OPEN", 28)
        self.closed_led = LEDIndicator("CLOSED", 28)
        self.moving_led = LEDIndicator("MOVING", 28)
        
        for led in (self.open_led, self.closed_led, self.moving_led):
            led_container = QHBoxLayout()
            led_container.addStretch()
            led_container.addWidget(led)
            led_container.addStretch()
            lid_layout.addLayout(led_container)
        lid_layout.addStretch()
        
        self.system_led_panel = QFrame()
        self.system_led_panel.setFixedSize(140, 380)
        self.system_led_panel.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2a2a2a, stop:0.5 #1a1a1a, stop:1 #0f0f0f);
                border: 2px solid #444;
                border-radius: 12px;
            }
        """)
        sys_layout = QVBoxLayout(self.system_led_panel)
        sys_layout.setSpacing(20)
        sys_layout.setContentsMargins(0, 25, 0, 25)
        sys_layout.setAlignment(Qt.AlignCenter)
        
        sys_label = QLabel("SYSTEM")
        sys_label.setAlignment(Qt.AlignCenter)
        sys_label.setFixedWidth(130)
        sys_label.setStyleSheet("""
            QLabel {
                color: #e0e0e0;
                font-size: 11pt;
                font-weight: bold;
                background-color: #1a1a1a;
                padding: 8px;
                border-radius: 6px;
                border: 1px solid #555;
            }
        """)
        sys_layout.addWidget(sys_label, 0, Qt.AlignCenter)
        
        self.system_led = LEDIndicator("READY", 28)
        self.fault_led = LEDIndicator("FAULT", 28)
        self.network_led = LEDIndicator("NETWORK", 28)
        
        for led in (self.system_led, self.fault_led, self.network_led):
            led_container = QHBoxLayout()
            led_container.addStretch()
            led_container.addWidget(led)
            led_container.addStretch()
            sys_layout.addLayout(led_container)
        sys_layout.addStretch()
    
    def _create_speaker_panel(self):
        from PyQt5.QtWidgets import QFrame
        
        speaker = QFrame()
        speaker.setFixedSize(140, 380)
        speaker.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2a2a2a, stop:0.5 #1a1a1a, stop:1 #0f0f0f);
                border: 2px solid #444;
                border-radius: 12px;
            }
        """)
        
        layout = QVBoxLayout(speaker)
        layout.setContentsMargins(0, 25, 0, 25)
        layout.setSpacing(15)
        layout.setAlignment(Qt.AlignCenter)
        
        label = QLabel("SPEAKER")
        label.setAlignment(Qt.AlignCenter)
        label.setFixedWidth(130)
        label.setStyleSheet("""
            QLabel {
                color: #e0e0e0;
                font-size: 11pt;
                font-weight: bold;
                background-color: #1a1a1a;
                padding: 8px;
                border-radius: 6px;
                border: 1px solid #555;
            }
        """)
        layout.addWidget(label, 0, Qt.AlignCenter)
        
        grill = QFrame()
        grill.setFixedSize(110, 270)
        grill_style = """
            QFrame {
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.7,
                    fx:0.5, fy:0.5, stop:0 #1a1a1a, stop:0.7 #0f0f0f, stop:1 #050505);
                border: 3px solid #333;
                border-radius: 20px;
            }
        """
        grill.setStyleSheet(grill_style)
        
        grill_layout = QVBoxLayout(grill)
        grill_layout.setContentsMargins(20, 20, 20, 20)
        grill_layout.setSpacing(8)
        
        for i in range(12):
            hole_row = QHBoxLayout()
            hole_row.setSpacing(8)
            for j in range(6):
                hole = QLabel()
                hole.setFixedSize(8, 8)
                hole.setStyleSheet("""
                    QLabel {
                        background-color: #000000;
                        border: 1px solid #0a0a0a;
                        border-radius: 4px;
                    }
                """)
                hole_row.addWidget(hole)
            grill_layout.addLayout(hole_row)
        
        layout.addWidget(grill, 0, Qt.AlignCenter)
        layout.addStretch()
        
        return speaker
    
    def _create_status_panel(self):
        self.status_panel = QFrame()
        self.status_panel.setFixedSize(340, 80)
        self.status_panel.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #1a1a1a, stop:0.1 #0f0f0f, stop:0.9 #050505, stop:1 #000000);
                border: 3px ridge #666;
                border-radius: 8px;
                padding: 10px;
            }
        """)
        status_layout = QVBoxLayout(self.status_panel)
        status_layout.setSpacing(4)
        
        self.manual_status = QLabel("Manual: IDLE")
        self.manual_status.setStyleSheet("QLabel { color: #00ff00; font-size: 8pt; font-family: monospace; "
                                        "background-color: transparent; border: none; "
                                        "text-shadow: 0 0 3px #00ff00; }")
        status_layout.addWidget(self.manual_status)
        
        self.buzzer_status = QLabel("Buzzer: OFF")
        self.buzzer_status.setStyleSheet("QLabel { color: #00ff00; font-size: 8pt; font-family: monospace; "
                                        "background-color: transparent; border: none; "
                                        "text-shadow: 0 0 3px #00ff00; }")
        status_layout.addWidget(self.buzzer_status)
        
        self.network_result = QLabel("Network: idle")
        self.network_result.setStyleSheet("QLabel { color: #00ff00; font-size: 8pt; font-family: monospace; "
                                         "background-color: transparent; "
                                         "text-shadow: 0 0 3px #00ff00; }")
        status_layout.addWidget(self.network_result)

    def _make_estop_button(self):
        from PyQt5.QtWidgets import QFrame, QVBoxLayout, QLabel
        
        container = QFrame()
        container.setFixedSize(130, 130)
        container.setStyleSheet("QFrame { background-color: #2a2a2a; border-radius: 65px; }")
        
        layout = QVBoxLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        
        button = QPushButton()
        button.setFixedSize(114, 114)
        style = """QPushButton {
            background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                fx:0.35, fy:0.35, stop:0 #ff3333, stop:0.4 #dd0000, stop:0.7 #aa0000, stop:1 #660000);
            color: white;
            border: 8px outset #8b0000;
            border-radius: 57px;
            font-weight: bold;
            font-size: 9pt;
        }
        QPushButton:pressed {
            background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                fx:0.4, fy:0.4, stop:0 #ff6666, stop:0.4 #ff0000, stop:0.7 #cc0000, stop:1 #880000);
            border: 8px inset #660000;
            padding-top: 3px;
        }"""
        button.setStyleSheet(style)
        button.setText("EMERGENCY\nSTOP")
        
        layout.addWidget(button)
        
        button.pressed.connect(self._estop_pressed)
        button.released.connect(self._estop_released)
        
        return container

    def _make_standard_button(self, text, released_color, pressed_color):
        button = QPushButton(text)
        button.setFixedSize(135, 50)
        style = (f"QPushButton {{ background-color: {released_color}; color: white; "
                f"border: 4px outset #555; border-radius: 6px; "
                f"font-weight: bold; font-size: 9pt; }}"
                f"QPushButton:pressed {{ background-color: {pressed_color}; "
                f"border: 4px inset #333; padding-top: 2px; }}")
        button.setStyleSheet(style)
        return button

    def _estop_pressed(self):
        self.estop_active = True
        self.set_enabled(False)
        self.estop_engaged.emit()

    def _estop_released(self):
        self.estop_active = False
        self.set_enabled(True)
        self.estop_released.emit()

    def set_enabled(self, enabled: bool):
        for btn in (self.btn_lid_open, self.btn_lid_close, self.btn_lid_stop,
                   self.btn_manual_open, self.btn_manual_close):
            btn.setEnabled(enabled)

    def set_manual_status(self, text: str):
        clean_text = text.replace("Manual control: ", "Manual: ")
        self.manual_status.setText(clean_text)

    def set_buzzer(self, alarm: bool):
        text = "Buzzer: ON" if alarm else "Buzzer: OFF"
        self.buzzer_status.setText(text)
        
        if alarm and not self.buzzer_active:
            self.buzzer_active = True
            self._start_buzzer()
        elif not alarm and self.buzzer_active:
            self.buzzer_active = False
            self._stop_buzzer()
    
    def _init_tts(self):
        """Initialize text-to-speech engine"""
        try:
            import pyttsx3
            self.tts_engine = pyttsx3.init()
            voices = self.tts_engine.getProperty('voices')
            for voice in voices:
                if 'female' in voice.name.lower():
                    self.tts_engine.setProperty('voice', voice.id)
                    break
            self.tts_engine.setProperty('rate', 150)
            self.tts_engine.setProperty('volume', 1.0)
        except:
            self.tts_engine = None
    
    def _start_buzzer(self):
        """Start voice alert only"""
        self._speak_error()
        self.buzzer_timer = QTimer()
        self.buzzer_timer.timeout.connect(self._speak_error)
        self.buzzer_timer.start(3000)  # Repeat voice every 3 seconds
    
    def _stop_buzzer(self):
        """Stop buzzer sound loop"""
        if self.buzzer_timer:
            self.buzzer_timer.stop()
            self.buzzer_timer = None
    
    def _speak_error(self):
        """Speak error message using TTS"""
        pass  # Error durumunda ses çıkarmıyor
    
    def speak_message(self, message: str):
        """Speak any message using TTS"""
        try:
            import sys, os
            if sys.platform == 'darwin':
                # macOS: Allison - most natural sounding voice
                os.system(f'say -v Allison -r 190 "{message}" &')
            elif sys.platform == 'linux':
                # Ubuntu: Try pico2wave (best), festival (better), then espeak
                if os.system('which pico2wave >/dev/null 2>&1') == 0:
                    os.system(f'pico2wave -l en-US -w /tmp/tts.wav "{message}" && aplay -q /tmp/tts.wav && rm /tmp/tts.wav &')
                elif os.system('which festival >/dev/null 2>&1') == 0:
                    os.system(f'echo "{message}" | festival --tts &')
                else:
                    os.system(f'espeak -v en+f3 -s 160 -p 50 "{message}" 2>/dev/null &')
            elif sys.platform == 'win32':
                # Windows: Use Microsoft Zira (natural female voice)
                if self.tts_engine:
                    import threading
                    def speak():
                        voices = self.tts_engine.getProperty('voices')
                        for voice in voices:
                            if 'zira' in voice.name.lower():
                                self.tts_engine.setProperty('voice', voice.id)
                                break
                        self.tts_engine.say(message)
                        self.tts_engine.runAndWait()
                    threading.Thread(target=speak, daemon=True).start()
        except Exception as e:
            print(f"TTS Error: {e}")
    
    def set_network_result(self, text: str):
        self.network_result.setText(text)

    def update_leds(self, state: int, error_code: int):
        self.open_led.set_color("#28a745" if state == 3 else "gray")
        self.closed_led.set_color("#17a2b8" if state == 4 else "gray")
        self.moving_led.set_color("#17a2b8" if state in (1, 2) else "gray")
        self.fault_led.set_color("#dc3545" if state == 6 else "gray")
        status_color = "#28a745" if state != 6 and error_code == 0 else "#dc3545"
        self.system_led.set_color(status_color)
        self.network_led.set_color("#28a745")
