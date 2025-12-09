from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class SystemPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QWidget { background-color: #0a0a0a; }")
        self._build_ui()
    
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Error Card
        self.error_card = self._create_card("⚠️", "OK", "System Error", "#28a745")
        
        # Network Card
        self.network_card = self._create_card("🌐", "OFFLINE", "Network Status", "#dc3545")
        
        layout.addWidget(self.error_card)
        layout.addWidget(self.network_card)
        layout.addStretch()
    
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
    
    def update(self, system: dict):
        error = system.get("error", {})
        network = system.get("network", {})
        
        error_code = error.get('code', 0)
        error_msg = error.get('msg', 'OK')
        
        if error_code == 0:
            error_text = "OK"
            error_color = "#28a745"
            error_icon = "✅"
        else:
            error_text = f"Code {error_code}: {error_msg}"
            error_color = "#dc3545"
            error_icon = "⚠️"
        
        self.error_card.findChild(QLabel, "icon").setText(error_icon)
        self.error_card.findChild(QLabel, "value").setText(error_text)
        self.error_card.findChild(QLabel, "value").setStyleSheet(f"QLabel {{ font-size: 15pt; color: {error_color}; font-weight: bold; background: transparent; border: none; }}")
        self.error_card.setStyleSheet(f"""
            QFrame {{
                background-color: #1a1a1a;
                border-left: 4px solid {error_color};
                border-radius: 6px;
            }}
        """)
        
        online = network.get("online", False)
        ip = network.get("ip", "0.0.0.0")
        
        if online:
            net_text = f"ONLINE · {ip}"
            net_color = "#28a745"
            net_icon = "✅"
        else:
            net_text = f"OFFLINE · {ip}"
            net_color = "#dc3545"
            net_icon = "❌"
        
        self.network_card.findChild(QLabel, "icon").setText(net_icon)
        self.network_card.findChild(QLabel, "value").setText(net_text)
        self.network_card.findChild(QLabel, "value").setStyleSheet(f"QLabel {{ font-size: 15pt; color: {net_color}; font-weight: bold; background: transparent; border: none; }}")
        self.network_card.setStyleSheet(f"""
            QFrame {{
                background-color: #1a1a1a;
                border-left: 4px solid {net_color};
                border-radius: 6px;
            }}
        """)
