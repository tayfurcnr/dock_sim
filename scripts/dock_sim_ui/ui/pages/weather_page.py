from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class WeatherPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QWidget { background-color: #0a0a0a; }")
        self.widgets = {}
        self._build_ui()
    
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Environment Cards Grid
        env_grid = QGridLayout()
        env_grid.setSpacing(10)
        
        temp_card = self._create_card("🌡️", "23.4°C", "Temperature", "#17a2b8")
        hum_card = self._create_card("💧", "56%", "Humidity", "#17a2b8")
        pres_card = self._create_card("🌫️", "1013 hPa", "Pressure", "#6c757d")
        light_card = self._create_card("☀️", "45k lx", "Light", "#ffc107")
        
        env_grid.addWidget(temp_card, 0, 0)
        env_grid.addWidget(hum_card, 0, 1)
        env_grid.addWidget(pres_card, 1, 0)
        env_grid.addWidget(light_card, 1, 1)
        
        # Wind Card
        wind_card = self._create_card("🌬️", "3.7 m/s @ 112°", "Wind", "#ffc107")
        
        # Rain Card
        rain_card = self._create_card("☀️", "DRY", "Rain Status", "#28a745")
        
        layout.addLayout(env_grid, 3)
        layout.addWidget(wind_card, 1)
        layout.addWidget(rain_card, 1)
        
        self.widgets["temp"] = temp_card.findChild(QLabel, "value")
        self.widgets["hum"] = hum_card.findChild(QLabel, "value")
        self.widgets["pres"] = pres_card.findChild(QLabel, "value")
        self.widgets["light"] = light_card.findChild(QLabel, "value")
        self.widgets["wind"] = wind_card.findChild(QLabel, "value")
        self.widgets["rain"] = rain_card.findChild(QLabel, "value")
        self.widgets["rain_icon"] = rain_card.findChild(QLabel, "icon")
    
    def _create_card(self, icon, value, label, color):
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: #1a1a1a;
                border-left: 4px solid {color};
                border-radius: 6px;
            }}
        """)
        layout = QHBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(8)
        
        left_layout = QVBoxLayout()
        left_layout.setSpacing(2)
        
        icon_label = QLabel(icon)
        icon_label.setObjectName("icon")
        icon_label.setStyleSheet("QLabel { font-size: 16pt; background: transparent; border: none; }")
        
        label_label = QLabel(label)
        label_label.setStyleSheet("QLabel { font-size: 9pt; color: #888; background: transparent; border: none; }")
        
        left_layout.addWidget(icon_label)
        left_layout.addWidget(label_label)
        
        value_label = QLabel(value)
        value_label.setObjectName("value")
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        value_label.setStyleSheet(f"QLabel {{ font-size: 15pt; color: {color}; font-weight: bold; background: transparent; border: none; }}")
        
        layout.addLayout(left_layout)
        layout.addStretch()
        layout.addWidget(value_label)
        
        return card
    
    def update(self, weather: dict):
        env = weather.get("env", {})
        wind = weather.get("wind", {})
        rain = weather.get("rain", {})
        
        temp = env.get('temp', 0)
        hum = env.get('hum', 0)
        pres = env.get('pres', 0)
        light = env.get('light', 0)
        
        self.widgets["temp"].setText(f"{temp:.1f}°C")
        self.widgets["hum"].setText(f"{hum:.0f}%")
        self.widgets["pres"].setText(f"{pres:.0f} hPa")
        self.widgets["light"].setText(f"{light/1000:.0f}k lx")
        
        speed = wind.get('speed', 0)
        direction = wind.get('dir', 0)
        self.widgets["wind"].setText(f"{speed:.1f} m/s @ {direction:.0f}°")
        
        rain_active = rain.get("active", False)
        rate = rain.get('rate', 0.0)
        rain_icon = "🌧️" if rain_active else "☀️"
        rain_text = f"RAIN {rate:.1f} mm/h" if rain_active else "DRY"
        
        self.widgets["rain_icon"].setText(rain_icon)
        self.widgets["rain"].setText(rain_text)
