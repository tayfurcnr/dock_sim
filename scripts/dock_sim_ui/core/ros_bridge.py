import yaml
import rospy
import actionlib
from pathlib import Path
from importlib import import_module
from PyQt5.QtCore import QObject, pyqtSignal
from std_msgs.msg import String
from dock_msgs.msg import Lid, WeatherEnv, WeatherWind, WeatherRain, Location, SystemError, SystemNetwork
try:
    from dock_msgs.msg import Charger
    CHARGER_AVAILABLE = True
except ImportError:
    CHARGER_AVAILABLE = False
    Charger = None
from dock_msgs.srv import LidControlTrigger
from dock_station_sim.srv import SetEnvironment, SetWind, TriggerWeatherEvent


class ROSBridge(QObject):
    telemetry_updated = pyqtSignal(dict)
    
    def __init__(self, config_path=None):
        super().__init__()
        self.data = {
            "telemetry": {
                "lid": {"state": 0},
                "weather": {"env": {}, "wind": {}, "rain": {}},
                "location": {},
                "system": {"error": {"code": 0, "msg": "OK"}, "network": {"online": False, "ip": "0.0.0.0"}},
                "charger": {"channels": []}
            }
        }
        
        if config_path is None:
            config_path = Path(__file__).parent.parent / "config" / "config.yaml"
        
        with open(config_path) as f:
            self.config = yaml.safe_load(f)
        
        self.services = {}
        self.publishers = {}
        self.actions = {}
        self.action_goal_types = {}
        self._init_subscribers()
        self._init_publishers()
        self._init_actions()
        self._init_services()
        self._env_service = None
        self._wind_service = None
        self._weather_event_service = None
        self._weather_service_clients = {}
    
    def _init_subscribers(self):
        topics = self.config['ros']['topics']
        rospy.Subscriber(topics['lid']['topic'], Lid, self._lid_callback)
        rospy.Subscriber(topics['weather_env']['topic'], WeatherEnv, self._weather_env_callback)
        rospy.Subscriber(topics['weather_wind']['topic'], WeatherWind, self._weather_wind_callback)
        rospy.Subscriber(topics['weather_rain']['topic'], WeatherRain, self._weather_rain_callback)
        rospy.Subscriber(topics['location']['topic'], Location, self._location_callback)
        rospy.Subscriber(topics['system_error']['topic'], SystemError, self._system_error_callback)
        rospy.Subscriber(topics['system_network']['topic'], SystemNetwork, self._system_network_callback)
        if CHARGER_AVAILABLE:
            rospy.Subscriber(topics['charger']['topic'], Charger, self._charger_callback)
    
    def _init_publishers(self):
        for name, pub_config in self.config['ros'].get('publishers', {}).items():
            self.publishers[name] = rospy.Publisher(pub_config['topic'], String, queue_size=10)
    
    def _init_actions(self):
        actions_cfg = self.config['ros'].get('actions', {})
        for name, action_config in actions_cfg.items():
            action_type = action_config.get('action_type')
            server = action_config.get('server')
            if not action_type or not server:
                rospy.logwarn(f"Action '{name}' missing action_type or server configuration")
                continue
            
            action_class, goal_class = self._load_action_classes(action_type)
            if not action_class or not goal_class:
                continue
            
            client = actionlib.SimpleActionClient(server, action_class)
            try:
                client.wait_for_server(rospy.Duration(1.0))
            except rospy.ROSException:
                rospy.logwarn(f"Action server '{server}' for '{name}' not available yet")
            self.actions[name] = client
            self.action_goal_types[name] = goal_class
    
    def _init_services(self):
        for name, srv_config in self.config['ros'].get('services', {}).items():
            try:
                service_class = self._load_service_class(srv_config.get('srv_type', 'dock_msgs/LidControlTrigger'))
                rospy.wait_for_service(srv_config['service'], timeout=2.0)
                self.services[name] = rospy.ServiceProxy(srv_config['service'], service_class)
            except Exception as e:
                rospy.loginfo(f"Service '{name}' not ready yet, will retry on first use")
                self.services[name] = None
    
    def _load_service_class(self, srv_type: str):
        """Dynamically import and return the service class for srv_type."""
        try:
            if '/' not in srv_type:
                raise ValueError(f"Invalid service type '{srv_type}'")
            package, service = srv_type.split('/')
            module = import_module(f"{package}.srv")
            return getattr(module, service)
        except Exception:
            return LidControlTrigger  # Silent fallback
    
    def call_action(self, action):
        if action not in self.config['ros']['button_actions']:
            return False, "Action not configured"
        
        action_config = self.config['ros']['button_actions'][action]
        action_type = action_config.get('type', 'service')
        
        if action_type == 'publish':
            publisher_name = action_config.get('publisher')
            message = action_config.get('message')
            
            publisher = self.publishers.get(publisher_name)
            if not publisher:
                return False, "Publisher not available"
            
            try:
                publisher.publish(String(data=message))
                return True, f"Published: {message}"
            except Exception as e:
                return False, str(e)
        elif action_type == 'action':
            return self._send_action_goal(action_config)
        else:
            service_name = action_config.get('service')
            service = self.services.get(service_name)
            if service is None:
                service = self._try_refresh_service(service_name)
            if not service:
                return False, "Service not available"
            
            try:
                from datetime import datetime
                transaction_id = f"ui_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
                ts = datetime.now().isoformat()

                request_kwargs = self._build_request_kwargs(action_config, transaction_id, ts)
                resp = service(**request_kwargs)
                return resp.success, resp.message
            except Exception as e:
                return False, str(e)
    
    def _send_action_goal(self, action_config):
        action_name = action_config.get('action')
        client = self.actions.get(action_name)
        if client is None:
            client = self._try_refresh_action(action_name)
        if not client:
            return False, "Action client not available"
        
        goal_class = self.action_goal_types.get(action_name)
        if not goal_class:
            return False, "Action goal type not available"
        
        try:
            from datetime import datetime
            from actionlib_msgs.msg import GoalStatus
            
            # Cancel previous goal if still active
            state = client.get_state()
            if state in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING]:
                client.cancel_goal()
                rospy.sleep(0.1)  # Brief wait for cancellation
            
            transaction_id = f"ui_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
            ts = datetime.now().isoformat()
            
            goal_kwargs = self._build_goal_kwargs(action_config, transaction_id, ts)
            goal_msg = goal_class()
            for key, value in goal_kwargs.items():
                setattr(goal_msg, key, value)
            
            if not client.wait_for_server(rospy.Duration(0.5)):
                return False, "Action server unavailable"
            
            client.send_goal(goal_msg)
            return True, "Command sent"
        except Exception as e:
            return False, str(e)
    
    def _build_request_kwargs(self, action_config, transaction_id, ts):
        """
        Build request keyword arguments for a service call.
        Supports explicit 'request' dict in config or falls back to legacy command params.
        """
        template = action_config.get('request')
        if template:
            request = {}
            for key, value in template.items():
                if isinstance(value, str):
                    if value == "{transaction_id}":
                        request[key] = transaction_id
                    elif value == "{timestamp}":
                        request[key] = ts
                    else:
                        request[key] = value
                else:
                    request[key] = value
            return request
        
        # Legacy path for numeric commands
        command = action_config['command']
        params = action_config.get('params', '')
        return {
            'command': command,
            'params': params,
            'transaction_id': transaction_id,
            'ts': ts
        }
    
    def _build_goal_kwargs(self, action_config, transaction_id, ts):
        goal_template = action_config.get('goal', {})
        goal = {}
        for key, value in goal_template.items():
            goal[key] = self._resolve_template_value(value, transaction_id, ts)
        
        goal.setdefault('transaction_id', transaction_id)
        goal.setdefault('ts', ts)
        goal.setdefault('params', goal.get('params', ''))
        return goal
    
    def _resolve_template_value(self, value, transaction_id, ts):
        if isinstance(value, str):
            if value == "{transaction_id}":
                return transaction_id
            if value == "{timestamp}":
                return ts
        return value
    
    def _try_refresh_service(self, service_name):
        """Attempt to (re)connect to a service that was unavailable at startup."""
        srv_config = self.config['ros']['services'].get(service_name)
        if not srv_config:
            return None
        try:
            service_class = self._load_service_class(srv_config.get('srv_type', 'dock_msgs/LidControlTrigger'))
            rospy.wait_for_service(srv_config['service'], timeout=0.5)
            proxy = rospy.ServiceProxy(srv_config['service'], service_class)
            self.services[service_name] = proxy
            return proxy
        except Exception:
            return None
    
    def _try_refresh_action(self, action_name):
        action_config = self.config['ros'].get('actions', {}).get(action_name)
        if not action_config:
            return None
        action_class, goal_class = self._load_action_classes(action_config.get('action_type', ''))
        if not action_class or not goal_class:
            return None
        try:
            client = actionlib.SimpleActionClient(action_config.get('server'), action_class)
            client.wait_for_server(rospy.Duration(0.5))
            self.actions[action_name] = client
            self.action_goal_types[action_name] = goal_class
            return client
        except Exception:
            return None
    
    def _load_action_classes(self, action_type):
        if not action_type or '/' not in action_type:
            rospy.logwarn(f"Invalid action_type '{action_type}'")
            return None, None
        try:
            package, action_name = action_type.split('/')
            module = import_module(f"{package}.msg")
            action_class = getattr(module, action_name)
            goal_class_name = action_name.replace('Action', 'Goal')
            goal_class = getattr(module, goal_class_name)
            return action_class, goal_class
        except Exception as e:
            rospy.logwarn(f"Failed to load action type '{action_type}': {e}")
            return None, None
    
    def _lid_callback(self, msg):
        self.data["telemetry"]["lid"] = {"state": msg.state, "percentage": msg.percentage, "ts": msg.ts}
        self.telemetry_updated.emit(self.data)
    
    def _weather_env_callback(self, msg):
        self.data["telemetry"]["weather"]["env"] = {"temp": msg.temp, "hum": msg.hum, "pres": msg.pres, "light": msg.light}
        self.telemetry_updated.emit(self.data)
    
    def _weather_wind_callback(self, msg):
        self.data["telemetry"]["weather"]["wind"] = {"speed": msg.speed, "dir": msg.dir}
        self.telemetry_updated.emit(self.data)
    
    def _weather_rain_callback(self, msg):
        self.data["telemetry"]["weather"]["rain"] = {"active": msg.active, "rate": msg.rate}
        self.telemetry_updated.emit(self.data)
    
    def _location_callback(self, msg):
        self.data["telemetry"]["location"] = {"lat": msg.lat, "lon": msg.lon, "alt": msg.alt, "ts": msg.ts}
        self.telemetry_updated.emit(self.data)
    
    def _system_error_callback(self, msg):
        self.data["telemetry"]["system"]["error"] = {"code": msg.code, "msg": msg.msg}
        self.telemetry_updated.emit(self.data)
    
    def _system_network_callback(self, msg):
        self.data["telemetry"]["system"]["network"] = {"ip": msg.ip, "online": msg.online}
        self.telemetry_updated.emit(self.data)
    
    def _charger_callback(self, msg):
        self.data["telemetry"]["charger"] = {"channels": msg.channels if hasattr(msg, 'channels') else []}
        self.telemetry_updated.emit(self.data)
    
    def set_weather_param(self, param_name, value):
        """
        Update simulated weather via sensor simulator services.
        - temperature/humidity/pressure -> SetEnvironment
        - wind_speed -> SetWind (value expected in km/h, converted to m/s)
        - rain_rate -> TriggerWeatherEvent (rain when value > 0, calm otherwise)
        """
        try:
            if param_name in ('temperature', 'humidity', 'pressure', 'light'):
                client = self._get_environment_service()
                if not client:
                    return False, "Environment service unavailable"
                req = {
                    'temperature': -1.0,
                    'humidity': -1.0,
                    'pressure': -1.0,
                    'light': -1
                }
                req[param_name] = value
                resp = client(**req)
                return resp.success, resp.message
            
            if param_name == 'wind_speed':
                client = self._get_wind_service()
                if not client:
                    return False, "Wind service unavailable"
                speed_mps = value / 3.6  # UI uses km/h
                resp = client(speed=speed_mps, direction=-1.0)
                return resp.success, resp.message
            
            if param_name == 'rain_rate':
                client = self._get_weather_event_service()
                if not client:
                    return False, "Weather event service unavailable"
                if value > 0:
                    duration = max(30.0, min(900.0, value * 30.0))
                    resp = client(event_type='rain', duration=duration)
                else:
                    resp = client(event_type='calm', duration=0.0)
                return resp.success, resp.message
            
            return False, f"Unsupported parameter: {param_name}"
        except Exception as e:
            rospy.logwarn(f"Weather service call failed for '{param_name}': {e}")
            return False, str(e)
    
    def _get_environment_service(self):
        if self._env_service is None:
            try:
                rospy.wait_for_service('/dock_station/sensor_sim/set_environment', timeout=1.0)
                self._env_service = rospy.ServiceProxy('/dock_station/sensor_sim/set_environment', SetEnvironment)
            except Exception:
                pass  # Silent retry
        return self._env_service
    
    def _get_wind_service(self):
        if self._wind_service is None:
            try:
                rospy.wait_for_service('/dock_station/sensor_sim/set_wind', timeout=1.0)
                self._wind_service = rospy.ServiceProxy('/dock_station/sensor_sim/set_wind', SetWind)
            except Exception:
                pass  # Silent retry
        return self._wind_service
    
    def _get_weather_event_service(self):
        if self._weather_event_service is None:
            try:
                rospy.wait_for_service('/dock_station/sensor_sim/trigger_weather_event', timeout=1.0)
                self._weather_event_service = rospy.ServiceProxy('/dock_station/sensor_sim/trigger_weather_event', TriggerWeatherEvent)
            except Exception:
                pass  # Silent retry
        return self._weather_event_service
