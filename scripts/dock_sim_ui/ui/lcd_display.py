from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QFrame, QLabel, QStackedWidget, QTextEdit, QVBoxLayout

from .pages import ChargerPage, LocationPage, SummaryPage, SystemPage, WeatherPage


class LCDDisplay(QFrame):
    page_selected = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setFixedSize(720, 408)
        self.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3a3a3a, stop:0.05 #2a2a2a, stop:0.95 #1a1a1a, stop:1 #0a0a0a);
                border: 12px outset #2a2a2a;
                border-radius: 8px;
            }
        """)
        
        inner = QFrame(self)
        inner.setGeometry(18, 18, 684, 372)
        inner.setStyleSheet("""
            QFrame {
                background-color: #000;
                border: 3px ridge #1a1a1a;
                border-radius: 4px;
            }
        """)
        
        layout = QVBoxLayout(inner)
        layout.setSpacing(2)
        layout.setContentsMargins(4, 4, 4, 4)

        self.status_banner = QLabel("SYSTEM READY")
        self.status_banner.setAlignment(Qt.AlignCenter)
        self.status_banner.setStyleSheet("""
            QLabel {
                font-size: 13pt;
                padding: 8px 15px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0d2838, stop:1 #11324b);
                border-radius: 6px;
                border: 2px solid #1a4d6b;
                color: #fff;
                font-weight: bold;
                letter-spacing: 1px;
            }
        """)
        layout.addWidget(self.status_banner)

        self.page_stack = QStackedWidget()
        
        self.summary_page = SummaryPage()
        self.weather_page = WeatherPage()
        self.location_page = LocationPage()
        self.system_page = SystemPage()
        self.charger_page = ChargerPage()
        
        self.page_stack.addWidget(self.summary_page)
        self.page_stack.addWidget(self.weather_page)
        self.page_stack.addWidget(self.location_page)
        self.page_stack.addWidget(self.system_page)
        self.page_stack.addWidget(self.charger_page)
        
        self.page_stack.setCurrentIndex(0)
        layout.addWidget(self.page_stack)

    def set_page(self, index: int):
        self.page_stack.setCurrentIndex(index)
        self.page_selected.emit(index)

    def update_summary(self, lid_state: int, lid_data: dict, weather: dict, system: dict, charger: dict, location: dict = None):
        self.summary_page.update(lid_state, lid_data, weather, system, charger, location)

    def update_weather(self, weather: dict):
        self.weather_page.update(weather)

    def update_location(self, location: dict):
        self.location_page.update(location)

    def update_system(self, system: dict):
        self.system_page.update(system)

    def update_charger(self, charger: dict):
        self.charger_page.update(charger)

    def append_log(self, message: str):
        pass

    def update_status_banner(self, text: str, color: str = "#11324b"):
        # Remove any emojis from text
        import re
        clean_text = re.sub(r'[^\w\s\-\(\):]', '', text).strip()
        
        # Color gradient mapping
        gradient_map = {
            "#28a745": ("#1a5c2e", "#28a745", "#2ecc71"),  # Green
            "#dc3545": ("#8b1e2b", "#dc3545", "#e74c3c"),  # Red
            "#17a2b8": ("#0d6674", "#17a2b8", "#1abc9c"),  # Cyan
            "#ffc107": ("#b8860b", "#ffc107", "#f39c12"),  # Yellow
            "#11324b": ("#0d2838", "#11324b", "#1a4d6b"),  # Blue (default)
        }
        
        grad_start, grad_mid, border_color = gradient_map.get(color, gradient_map["#11324b"])
        
        self.status_banner.setText(clean_text)
        self.status_banner.setStyleSheet(f"""
            QLabel {{
                font-size: 13pt;
                padding: 8px 15px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {grad_start}, stop:1 {grad_mid});
                border-radius: 6px;
                border: 2px solid {border_color};
                color: #fff;
                font-weight: bold;
                letter-spacing: 1px;
            }}
        """)
