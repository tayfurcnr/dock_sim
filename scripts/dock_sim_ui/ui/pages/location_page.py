from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget


class LocationPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("QWidget { background-color: #0a0a0a; }")
        self.lat = None
        self.lon = None
        self.map_loaded = False
        self._build_ui()
    
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.coord_label = QLabel("📍 Waiting for location data...")
        self.coord_label.setAlignment(Qt.AlignCenter)
        self.coord_label.setFixedHeight(35)
        self.coord_label.setStyleSheet("""
            QLabel {
                font-size: 12pt;
                color: #17a2b8;
                background-color: #1a1a1a;
                border-left: 4px solid #17a2b8;
                border-radius: 6px;
                padding: 5px 10px;
                font-weight: bold;
            }
        """)
        layout.addWidget(self.coord_label)
        
        self.map_view = QWebEngineView()
        self.map_view.setStyleSheet("QWebEngineView { background-color: #0a0a0a; border: 1px solid #333; }")
        layout.addWidget(self.map_view)
    
    def _load_map(self):
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <style>
                body {{ margin: 0; padding: 0; }}
                #map {{ width: 100%; height: 100vh; }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map').setView([{self.lat}, {self.lon}], 13);
                L.tileLayer('https://{{s}}.basemaps.cartocdn.com/dark_all/{{z}}/{{x}}/{{y}}{{r}}.png', {{
                    maxZoom: 19,
                    attribution: '© OpenStreetMap © CartoDB'
                }}).addTo(map);
                
                var marker = L.marker([{self.lat}, {self.lon}]).addTo(map);
                marker.bindPopup("<b>Current Location</b>").openPopup();
                
                window.updateLocation = function(lat, lng) {{
                    marker.setLatLng([lat, lng]);
                    map.setView([lat, lng]);
                }};
            </script>
        </body>
        </html>
        """
        self.map_view.setHtml(html)
    
    def update(self, location: dict):
        self.lat = location.get("lat", 37.7749)
        self.lon = location.get("lon", -122.4194)
        alt = location.get("alt", 0.0)
        self.coord_label.setText(f"📍 Lat: {self.lat:.5f}° | Lon: {self.lon:.5f}° | Alt: {alt:.1f}m")
        
        if not self.map_loaded:
            self._load_map()
            self.map_loaded = True
        else:
            self.map_view.page().runJavaScript(f"if(window.updateLocation) updateLocation({self.lat}, {self.lon});")
