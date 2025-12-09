from copy import deepcopy

LID_STATES = {
    0: "UNKNOWN", 1: "OPENING", 2: "CLOSING",
    3: "OPEN", 4: "CLOSED", 5: "STOPPED", 6: "ERROR"
}

LID_ICONS = {
    0: "⚪", 1: "🔁", 2: "🔁", 3: "🔓",
    4: "🔒", 5: "🟥", 6: "⚠"
}

SAMPLE_TELEMETRY = {
    "telemetry": {
        "weather": {
            "env": {
                "temp": 23.4,
                "hum": 56.7,
                "pres": 1013.8,
                "light": 45230,
                "ts": "2023-10-05T12:34:56Z",
            },
            "wind": {
                "speed": 3.7,
                "dir": 112.0,
                "ts": "2023-10-05T12:34:56Z",
            },
            "rain": {"active": False, "rate": 0.0, "ts": "2023-10-05T12:34:56Z"},
        },
        "lid": {"state": 3, "percentage": 100, "ts": "2023-10-05T12:34:56Z"},
        "location": {
            "lat": 37.7749,
            "lon": -122.4194,
            "alt": 15.2,
            "ts": "2023-10-05T12:34:56Z",
        },
        "system": {
            "error": {"code": 0, "msg": "OK", "ts": "2023-10-05T12:34:56Z"},
            "network": {"ip": "192.168.1.50", "online": True, "ts": "2023-10-05T12:34:56Z"},
        },
        "charger": {
            "channels": [
                {
                    "channel": 1,
                    "status": "RUNNING",
                    "error": "",
                    "total_voltage_v": 24.85,
                    "cell_voltages": [4.15, 4.14, 4.14, 4.14, 4.14, 4.14],
                    "temp_cutoff_limit_c": 45,
                    "ts": "2023-10-05T12:34:56Z",
                },
                {
                    "channel": 2,
                    "status": "IDLE",
                    "error": "OK",
                    "total_voltage_v": 22.2,
                    "cell_voltages": [3.7, 3.7, 3.7, 3.7, 3.7, 3.7],
                    "temp_cutoff_limit_c": 45,
                    "ts": "2023-10-05T12:34:50Z",
                },
            ]
        },
    }
}


def sample_telemetry():
    return deepcopy(SAMPLE_TELEMETRY)
