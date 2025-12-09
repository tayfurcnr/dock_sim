from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget


class ChargerPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QWidget { background-color: #0a0a0a; }")
        self._build_ui()
    
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        self.channels_layout = QVBoxLayout()
        self.channels_layout.setSpacing(10)
        layout.addLayout(self.channels_layout)
        layout.addStretch()
    
    def update(self, charger: dict):
        while self.channels_layout.count():
            if widget := self.channels_layout.takeAt(0).widget():
                widget.deleteLater()
        
        channels = charger.get("channels", [])
        
        if not channels:
            no_data = QLabel("No charger data available")
            no_data.setAlignment(Qt.AlignCenter)
            no_data.setStyleSheet("QLabel { font-size: 12pt; color: #666; }")
            self.channels_layout.addWidget(no_data)
            return
        
        for entry in channels:
            card = self._create_channel_card(entry)
            self.channels_layout.addWidget(card)
    
    def _create_channel_card(self, entry):
        ch = entry.get('channel', 0)
        status = entry.get('status', 'UNKNOWN')
        voltage = entry.get('total_voltage_v', 0)
        error_text = entry.get("error", "")
        
        if status == "RUNNING":
            color = "#28a745"
            icon = "🔋"
        elif status == "IDLE":
            color = "#6c757d"
            icon = "⏸️"
        else:
            color = "#dc3545"
            icon = "⚠️"
        
        card = QFrame()
        card.setStyleSheet(f"""
            QFrame {{
                background-color: #1a1a1a;
                border-left: 4px solid {color};
                border-radius: 6px;
            }}
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(5)
        
        # Header
        header_layout = QHBoxLayout()
        
        left_header = QVBoxLayout()
        left_header.setSpacing(2)
        
        icon_label = QLabel(icon)
        icon_label.setStyleSheet("QLabel { font-size: 16pt; background: transparent; border: none; }")
        
        ch_label = QLabel(f"Channel {ch}")
        ch_label.setStyleSheet("QLabel { font-size: 9pt; color: #888; background: transparent; border: none; }")
        
        left_header.addWidget(icon_label)
        left_header.addWidget(ch_label)
        
        right_header = QVBoxLayout()
        right_header.setSpacing(2)
        
        status_label = QLabel(status)
        status_label.setAlignment(Qt.AlignRight)
        status_label.setStyleSheet(f"QLabel {{ font-size: 13pt; color: {color}; font-weight: bold; background: transparent; border: none; }}")
        
        voltage_label = QLabel(f"{voltage:.2f}V")
        voltage_label.setAlignment(Qt.AlignRight)
        voltage_label.setStyleSheet("QLabel { font-size: 11pt; color: #aaa; background: transparent; border: none; }")
        
        right_header.addWidget(status_label)
        right_header.addWidget(voltage_label)
        
        header_layout.addLayout(left_header)
        header_layout.addStretch()
        header_layout.addLayout(right_header)
        
        layout.addLayout(header_layout)
        
        # Details
        details_layout = QHBoxLayout()
        details_layout.setSpacing(15)
        
        if cells := entry.get("cell_voltages", []):
            cell_label = QLabel(f"Cells: {min(cells):.2f}-{max(cells):.2f}V")
            cell_label.setStyleSheet("QLabel { font-size: 9pt; color: #888; background: transparent; border: none; }")
            details_layout.addWidget(cell_label)
        
        temp_limit = entry.get('temp_cutoff_limit_c', 0)
        temp_label = QLabel(f"Cutoff: {temp_limit}°C")
        temp_label.setStyleSheet("QLabel { font-size: 9pt; color: #888; background: transparent; border: none; }")
        details_layout.addWidget(temp_label)
        
        details_layout.addStretch()
        layout.addLayout(details_layout)
        
        # Error
        if error_text and error_text != "OK":
            error_label = QLabel(f"⚠️ {error_text}")
            error_label.setStyleSheet("QLabel { font-size: 10pt; color: #dc3545; background: transparent; border: none; font-weight: bold; }")
            layout.addWidget(error_label)
        
        return card
