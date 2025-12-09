from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class SummaryPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QWidget { background-color: #0a0a0a; }")
        self.widgets = {}
        self._build_ui()
    
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Lid Status Card
        lid_card = QFrame()
        lid_card.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1a1a1a, stop:1 #0f0f0f);
                border: 2px solid #333;
                border-radius: 8px;
            }
        """)
        lid_layout = QHBoxLayout(lid_card)
        lid_layout.setContentsMargins(15, 12, 15, 12)
        lid_layout.setSpacing(15)
        
        left_lid = QVBoxLayout()
        left_lid.setSpacing(3)
        
        lid_icon = QLabel("🔓")
        lid_icon.setStyleSheet("QLabel { font-size: 32pt; background: transparent; border: none; }")
        
        lid_label = QLabel("Lid Status")
        lid_label.setStyleSheet("QLabel { font-size: 10pt; color: #888; background: transparent; border: none; }")
        
        left_lid.addWidget(lid_icon)
        left_lid.addWidget(lid_label)
        
        right_lid = QVBoxLayout()
        right_lid.setSpacing(3)
        
        lid_state = QLabel("OPEN")
        lid_state.setAlignment(Qt.AlignRight)
        lid_state.setStyleSheet("QLabel { font-size: 24pt; color: #28a745; font-weight: bold; background: transparent; border: none; }")
        
        lid_percent = QLabel("100%")
        lid_percent.setAlignment(Qt.AlignRight)
        lid_percent.setStyleSheet("QLabel { font-size: 14pt; color: #aaa; background: transparent; border: none; }")
        
        right_lid.addWidget(lid_state)
        right_lid.addWidget(lid_percent)
        
        lid_layout.addLayout(left_lid)
        lid_layout.addStretch()
        lid_layout.addLayout(right_lid)
        
        # Environment Cards
        env_grid = QGridLayout()
        env_grid.setSpacing(10)
        
        temp_card = self._create_card("🌡️", "23.4°C", "Temperature", "#17a2b8")
        hum_card = self._create_card("💧", "56%", "Humidity", "#17a2b8")
        light_card = self._create_card("☀️", "45k lx", "Light", "#ffc107")
        press_card = self._create_card("🌪️", "1013 hPa", "Pressure", "#6c757d")
        
        env_grid.addWidget(temp_card, 0, 0)
        env_grid.addWidget(hum_card, 0, 1)
        env_grid.addWidget(light_card, 1, 0)
        env_grid.addWidget(press_card, 1, 1)
        
        # Network Status
        net_card = QFrame()
        net_card.setStyleSheet("""
            QFrame {
                background-color: #1a1a1a;
                border-left: 4px solid #6c757d;
                border-radius: 6px;
            }
        """)
        net_layout = QHBoxLayout(net_card)
        net_layout.setContentsMargins(10, 8, 10, 8)
        net_layout.setSpacing(8)
        
        left_net = QVBoxLayout()
        left_net.setSpacing(2)
        
        net_icon = QLabel("🌐")
        net_icon.setStyleSheet("QLabel { font-size: 16pt; background: transparent; border: none; }")
        
        net_label = QLabel("Network")
        net_label.setStyleSheet("QLabel { font-size: 9pt; color: #888; background: transparent; border: none; }")
        
        left_net.addWidget(net_icon)
        left_net.addWidget(net_label)
        
        right_net = QVBoxLayout()
        right_net.setSpacing(2)
        
        net_status = QLabel("✅ ONLINE")
        net_status.setAlignment(Qt.AlignRight)
        net_status.setStyleSheet("QLabel { font-size: 11pt; color: #28a745; font-weight: bold; background: transparent; border: none; }")
        
        net_ip = QLabel("192.168.1.50")
        net_ip.setAlignment(Qt.AlignRight)
        net_ip.setStyleSheet("QLabel { font-size: 10pt; color: #aaa; background: transparent; border: none; }")
        
        right_net.addWidget(net_status)
        right_net.addWidget(net_ip)
        
        net_layout.addLayout(left_net)
        net_layout.addStretch()
        net_layout.addLayout(right_net)
        
        layout.addWidget(lid_card, 1)
        layout.addLayout(env_grid, 3)
        layout.addWidget(net_card, 1)
        
        self.widgets["lid_icon"] = lid_icon
        self.widgets["lid_state"] = lid_state
        self.widgets["lid_percent"] = lid_percent
        self.widgets["temp"] = temp_card.findChild(QLabel, "value")
        self.widgets["hum"] = hum_card.findChild(QLabel, "value")
        self.widgets["light"] = light_card.findChild(QLabel, "value")
        self.widgets["press"] = press_card.findChild(QLabel, "value")
        self.widgets["net_status"] = net_status
        self.widgets["net_ip"] = net_ip
    
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
    
    def update(self, lid_state: int, lid_data: dict, weather: dict, system: dict, charger: dict, location: dict = None):
        states = {0: "UNKNOWN", 1: "OPENING", 2: "CLOSING", 3: "OPEN", 4: "CLOSED", 5: "STOPPED", 6: "ERROR"}
        colors = {0: "#888", 1: "#17a2b8", 2: "#17a2b8", 3: "#28a745", 4: "#ffc107", 5: "#dc3545", 6: "#dc3545"}
        icons = {0: "⚪", 1: "🔁", 2: "🔁", 3: "🔓", 4: "🔒", 5: "🟥", 6: "⚠️"}
        
        state_text = states.get(lid_state, "UNKNOWN")
        state_color = colors.get(lid_state, "#888")
        state_icon = icons.get(lid_state, "⚪")
        
        self.widgets["lid_icon"].setText(state_icon)
        self.widgets["lid_state"].setText(state_text)
        self.widgets["lid_state"].setStyleSheet(f"QLabel {{ font-size: 24pt; color: {state_color}; font-weight: bold; background: transparent; border: none; }}")
        
        percentage = lid_data.get("percentage", 0)
        self.widgets["lid_percent"].setText(f"{percentage}%")
        
        env = weather.get("env", {})
        self.widgets["temp"].setText(f"{env.get('temp', 0):.1f}°C")
        self.widgets["hum"].setText(f"{env.get('hum', 0):.0f}%")
        self.widgets["light"].setText(f"{env.get('light', 0)/1000:.0f}k lx")
        self.widgets["press"].setText(f"{env.get('pres', 0):.0f} hPa")
        
        net = system.get("network", {})
        online = net.get("online", False)
        ip = net.get("ip", "0.0.0.0")
        
        status_text = "✅ ONLINE" if online else "❌ OFFLINE"
        status_color = "#28a745" if online else "#dc3545"
        
        self.widgets["net_status"].setText(status_text)
        self.widgets["net_status"].setStyleSheet(f"QLabel {{ font-size: 12pt; color: {status_color}; font-weight: bold; background: transparent; border: none; }}")
        self.widgets["net_ip"].setText(ip)
