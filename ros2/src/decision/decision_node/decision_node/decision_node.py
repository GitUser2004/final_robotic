#!/usr/bin/env python3
"""
Robot Hospital - ESCANEO DE OBSTÁCULOS 
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String, Int16
import time
import requests
import threading

class VisionNavigationNode(Node):
    def __init__(self):
        super().__init__('vision_navigation_node')
        
        # ═══════════════════════════════════════════════════════════
        # CONFIGURACIÓN ESP32
        # ═══════════════════════════════════════════════════════════
        self.ESP32_IP = "192.168.100.105"
        self.CAM_IP = "192.168.100.101"
        self.vision_thread = None
        self.vision_running = False
        
        # Datos de visión
        self.color_detectado = False
        self.color_area = 0
        self.color_posicion_x = 0
        self.ultimo_dato_vision = time.time()
        
        # Control de cámara
        self.camera_consecutive_errors = 0
        self.camera_max_errors = 5
        self.camera_available = True
        
        # Control de prioridades
        self.servo_locked = False
        self.vision_paused = False
        
        # ═══════════════════════════════════════════════════════════
        # SUSCRIPCIONES Y PUBLICADORES ROS2
        # ═══════════════════════════════════════════════════════════
        self.distance_front_sub = self.create_subscription(
            Float32, '/distance_front', self.distance_front_callback, 10)
        self.distance_left_sub = self.create_subscription(
            Float32, '/distance_left', self.distance_left_callback, 10)
        self.distance_right_sub = self.create_subscription(
            Float32, '/distance_right', self.distance_right_callback, 10)
        self.odom_sub = self.create_subscription(
            Float32, '/odom', self.odom_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_cmd_pub = self.create_publisher(Bool, '/scan_command', 10)
        self.servo_angle_pub = self.create_publisher(Int16, '/servo_angle', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # ═══════════════════════════════════════════════════════════
        # PARÁMETROS DE NAVEGACIÓN
        # ═══════════════════════════════════════════════════════════
        self.obstacle_threshold = 0.22
        self.safe_distance = 0.28
        
        self.cruise_speed = 0.35
        self.approach_speed = 0.25
        self.turn_speed_45 = 0.72
        self.turn_duration_45 = 1.2
        self.turn_speed_align = 0.40
        self.reverse_speed = -0.30
        self.reverse_duration = 1.5
        
        # ═══════════════════════════════════════════════════════════
        # PARÁMETROS DE ESCANEO MEJORADOS
        # ═══════════════════════════════════════════════════════════
        self.search_interval = 5.0
        self.scan_speed = 10
        self.scan_delay = 0.3
        self.vision_timeout = 2.0
        
        self.area_detected_threshold = 800
        self.area_reached_threshold = 25000
        
        self.centered_time = None
        self.required_centered_duration = 0.3
        self.area_when_centered = 0
        self.min_area_for_centering = 500
        
        self.last_area_log = 0
        
        # ═══════════════════════════════════════════════════════════
        # SISTEMA DE MISIONES
        # ═══════════════════════════════════════════════════════════
        self.mission_state = 'IDLE'
        self.color_objetivo = None
        self.habitacion_visitada = None
        
        # ═══════════════════════════════════════════════════════════
        # DETECCIÓN DE ATASCO Y MOTORES
        # ═══════════════════════════════════════════════════════════
        self.odom_distance = 0.0
        self.last_odom_check = 0.0
        self.last_odom_update_time = time.time()
        self.odom_stuck_start = None
        self.stuck_threshold = 0.008
        self.stuck_timeout = 4.0
        self.odom_initialized = False
        self.odom_init_time = None
        self.odom_warmup_time = 6.0
        self.last_speed_command = 0.0
        
        self.motor_failure_detected = False
        self.motor_recovery_attempts = 0
        self.max_motor_recovery_attempts = 3
        self.odom_timeout = 5.0
        
        # ═══════════════════════════════════════════════════════════
        # ESTADO DEL ROBOT
        # ═══════════════════════════════════════════════════════════
        self.distance_front = 2.0
        self.distance_left = 2.0
        self.distance_right = 2.0
        
        # Control de escaneo mejorado
        self.scan_data_count = 0
        self.scan_data_complete = False
        self.last_scan_data = {
            'left': 2.0,
            'right': 2.0,
            'front': 2.0
        }
        
        self.nav_state = 'WAITING'
        
        # Control de búsqueda visual
        self.last_search_time = 0
        self.scan_angle = 90
        self.scan_direction = 1
        
        # 🆕 Control de escaneo robusto
        self.waiting_scan = False
        self.scan_start_time = 0
        self.scan_timeout = 2.5  # 🔧 Reducido de 3.0 a 2.5s
        self.scan_request_time = 0
        self.scan_retry_count = 0
        self.max_scan_retries = 2
        self.last_scan_attempt = 0
        
        # Control de giros
        self.turn_counter = 0
        self.turn_direction = None
        self.reverse_counter = 0
        
        # PREVENCIÓN DE LOOPS
        self.last_stuck_position = None
        self.stuck_recovery_attempts = 0
        self.max_recovery_attempts = 2
        
        self.get_logger().info("╔═══════════════════════════════════════════════╗")
        self.get_logger().info("║  ROBOT HOSPITAL - ESCANEO ROBUSTO            ║")
        self.get_logger().info("╚═══════════════════════════════════════════════╝")
        self.get_logger().info(f" Esperando comando START_MISSION...\n")
        
        self.start_vision_thread()
        self.mission_thread = threading.Thread(target=self.mission_polling_loop, daemon=True)
        self.mission_thread.start()

    def start_vision_thread(self):
        if not self.vision_running:
            self.vision_running = True
            self.vision_thread = threading.Thread(target=self.vision_processing_loop, daemon=True)
            self.vision_thread.start()

    def mission_polling_loop(self):
        while self.vision_running:
            try:
                r = requests.get(f"http://{self.ESP32_IP}/mission_state", timeout=1)
                if r.status_code == 200:
                    esp_state = r.text.strip().upper()
                    
                    if esp_state == "BUSCANDO" and self.mission_state == 'IDLE':
                        r2 = requests.get(f"http://{self.ESP32_IP}/target", timeout=1)
                        if r2.status_code == 200:
                            color = r2.text.strip().upper()
                            if color in ["ROJO", "VERDE", "AZUL"]:
                                self.color_objetivo = color
                                self.mission_state = 'GOING_TO_ROOM'
                                self.nav_state = 'SEARCHING'
                                self.last_search_time = time.time()
                                self.vision_paused = False
                                self.camera_consecutive_errors = 0
                                self.camera_available = True
                                self.stuck_recovery_attempts = 0
                                self.motor_failure_detected = False
                                self.motor_recovery_attempts = 0
                                self.scan_retry_count = 0  
                                self.get_logger().info(f"MISIÓN: Ir a habitación {color}")
                    
                    elif esp_state == "PAUSA" and self.mission_state == 'GOING_TO_ROOM':
                        self.mission_state = 'AT_ROOM'
                        self.habitacion_visitada = self.color_objetivo
                        self.nav_state = 'WAITING'
                        self.vision_paused = True
                        self.get_logger().info(f"Llegó a habitación {self.habitacion_visitada}")
                    
                    elif esp_state == "VOLVER" and self.mission_state == 'AT_ROOM':
                        self.color_objetivo = "AMARILLO"
                        self.mission_state = 'RETURNING_TO_WAREHOUSE'
                        self.nav_state = 'SEARCHING'
                        self.last_search_time = time.time()
                        self.vision_paused = False
                        self.camera_consecutive_errors = 0
                        self.camera_available = True
                        self.stuck_recovery_attempts = 0
                        self.motor_failure_detected = False
                        self.motor_recovery_attempts = 0
                        self.scan_retry_count = 0  
                        if hasattr(self, '_yellow_close_confirmed'):
                            delattr(self, '_yellow_close_confirmed')
                        self.get_logger().info(f"RETORNO: Buscando almacén (AMARILLO)")
                    
                    elif esp_state == "IDLE" and self.mission_state == 'RETURNING_TO_WAREHOUSE':
                        self.mission_state = 'AT_WAREHOUSE'
                        self.nav_state = 'WAITING'
                        self.color_objetivo = None
                        self.vision_paused = True
                        self.get_logger().info(f"Llegó a almacén")
                        time.sleep(1)
                        self.mission_state = 'IDLE'
                        
            except:
                pass
            
            time.sleep(0.3)

    def vision_processing_loop(self):
        import cv2
        import numpy as np
        
        CAM_URL = f"http://{self.CAM_IP}/capture"
        kernel = np.ones((5,5), np.uint8)
        
        DP = 1.2
        MIN_DIST = 60
        P1 = 120
        P2 = 18
        MIN_R = 12
        MAX_R = 180
        
        while self.vision_running:
            if (self.vision_paused or 
                self.servo_locked or 
                self.color_objetivo is None or 
                self.nav_state in ['WAITING', 'OBSTACLE_STOP', 'OBSTACLE_SCAN', 
                                   'TURNING', 'STUCK_REVERSE']):
                time.sleep(0.3)
                continue
            
            if self.camera_consecutive_errors >= self.camera_max_errors:
                if not self.camera_available:
                    time.sleep(2.0)
                    self.camera_consecutive_errors = 0
                    self.get_logger().info("📷 Reintentando cámara...")
                self.camera_available = False
            
            try:
                r = requests.get(CAM_URL, timeout=2)
                if r.status_code != 200:
                    self.camera_consecutive_errors += 1
                    continue
                
                try:
                    img = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
                except:
                    self.camera_consecutive_errors += 1
                    time.sleep(0.5)
                    continue
                
                if img is None:
                    self.camera_consecutive_errors += 1
                    continue
                
                if self.camera_consecutive_errors > 0:
                    self.get_logger().info("✓ Cámara reconectada")
                self.camera_consecutive_errors = 0
                self.camera_available = True
                
                h, w = img.shape[:2]
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                hsv = cv2.GaussianBlur(hsv, (5,5), 0)
                
                if self.color_objetivo == "ROJO":
                    m1 = cv2.inRange(hsv, (0,120,70), (10,255,255))
                    m2 = cv2.inRange(hsv, (170,120,70), (180,255,255))
                    mask = m1 + m2
                elif self.color_objetivo == "VERDE":
                    mask = cv2.inRange(hsv, (36,50,70), (89,255,255))
                elif self.color_objetivo == "AZUL":
                    mask = cv2.inRange(hsv, (90,50,70), (128,255,255))
                elif self.color_objetivo == "AMARILLO":
                    mask = cv2.inRange(hsv, (20,100,150), (30,255,255))
                else:
                    continue
                
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                mask_blur = cv2.GaussianBlur(mask, (9, 9), 2)
                circles = cv2.HoughCircles(
                    mask_blur, cv2.HOUGH_GRADIENT,
                    dp=DP, minDist=MIN_DIST, param1=P1, param2=P2,
                    minRadius=MIN_R, maxRadius=MAX_R
                )
                
                if circles is not None:
                    circles = np.uint16(np.around(circles[0]))
                    best = max(circles, key=lambda c: c[2])
                    x, y, rad = best
                    
                    self.color_detectado = True
                    self.color_area = int(np.pi * (rad ** 2))
                    
                    if self.color_objetivo == "AMARILLO":
                        if self.color_area > 15000 and not hasattr(self, '_yellow_close_confirmed'):
                            self.get_logger().warn(f"Amarillo: área sospechosa {self.color_area} - requiere verificación")
                            self.color_area = int(self.color_area * 0.3)
                    
                    center_x = w / 2
                    offset = x - center_x
                    threshold = w * 0.15
                    
                    if offset < -threshold:
                        self.color_posicion_x = -1
                    elif offset > threshold:
                        self.color_posicion_x = 1
                    else:
                        self.color_posicion_x = 0
                    
                    self.ultimo_dato_vision = time.time()
                else:
                    self.color_detectado = False
                    self.color_area = 0
                
            except requests.exceptions.Timeout:
                self.camera_consecutive_errors += 1
                if self.camera_consecutive_errors == self.camera_max_errors:
                    self.get_logger().warn(f"Cámara no responde")
            except Exception as e:
                self.camera_consecutive_errors += 1
                if "JPEG" not in str(e) and "extraneous bytes" not in str(e):
                    self.get_logger().error(f"Error visión: {e}")
            
            time.sleep(0.2)

    def distance_front_callback(self, msg):
        self.distance_front = msg.data
        if self.waiting_scan:
            self.last_scan_data['front'] = msg.data
            self.scan_data_count += 1
            self.get_logger().info(f"Escaneo recibido: FRENTE = {msg.data:.2f}m (datos: {self.scan_data_count}/3)")

    def distance_left_callback(self, msg):
        self.distance_left = msg.data
        if self.waiting_scan:
            self.last_scan_data['left'] = msg.data
            self.scan_data_count += 1
            self.get_logger().info(f"Escaneo recibido: IZQUIERDA = {msg.data:.2f}m (datos: {self.scan_data_count}/3)")

    def distance_right_callback(self, msg):
        self.distance_right = msg.data
        if self.waiting_scan:
            self.last_scan_data['right'] = msg.data
            self.scan_data_count += 1
            self.get_logger().info(f"Escaneo recibido: DERECHA = {msg.data:.2f}m (datos: {self.scan_data_count}/3)")

    def odom_callback(self, msg):
        prev_odom = self.odom_distance
        self.odom_distance = msg.data
        
        if msg.data != prev_odom:
            self.last_odom_update_time = time.time()
            if self.motor_failure_detected:
                self.get_logger().info("✓ Motores respondiendo nuevamente")
                self.motor_failure_detected = False
                self.motor_recovery_attempts = 0
        
        if not self.odom_initialized and msg.data > 0.0:
            self.odom_initialized = True
            self.odom_init_time = time.time()
            self.last_odom_check = msg.data
        elif self.odom_initialized and msg.data == 0.0:
            self.odom_initialized = False
            self.odom_init_time = None
            self.odom_stuck_start = None

    def set_servo_angle(self, angle):
        if not self.servo_locked:
            msg = Int16()
            msg.data = int(angle)
            self.servo_angle_pub.publish(msg)
            self.scan_angle = angle

    def request_obstacle_scan(self):
        """🔧 Solicitar escaneo con logging detallado"""
        self.servo_locked = True
        self.vision_paused = True
        
        self.scan_data_count = 0
        self.scan_data_complete = False
        
        msg = Bool()
        msg.data = True
        self.scan_cmd_pub.publish(msg)
        self.waiting_scan = True
        self.scan_start_time = time.time()
        self.scan_request_time = time.time()
        
        self.get_logger().info("Escaneo solicitado al ESP32...")
        self.get_logger().info(f"   → Timeout: {self.scan_timeout}s")
        self.get_logger().info(f"   → Intento: {self.scan_retry_count + 1}/{self.max_scan_retries + 1}")

    def check_scan_complete(self):
        """Verificar si recibimos las 3 lecturas"""
        return self.scan_data_count >= 3

    def analyze_obstacle_scan(self):
        """Análisis: siempre girar hacia el lado más libre"""
        front = self.last_scan_data['front']
        left = self.last_scan_data['left']
        right = self.last_scan_data['right']
        
        self.get_logger().info(
            f"Escaneo completo → F={front:.2f}m, L={left:.2f}m, R={right:.2f}m"
        )
        
        if left > right + 0.10:
            self.get_logger().info(f"↰ Decisión: IZQUIERDA más libre ({left:.2f}m vs {right:.2f}m)")
            return 'LEFT'
        elif right > left + 0.10:
            self.get_logger().info(f"↱ Decisión: DERECHA más libre ({right:.2f}m vs {left:.2f}m)")
            return 'RIGHT'
        else:
            if left > right:
                self.get_logger().info(f"↰ Decisión: IZQUIERDA (empate, {left:.2f}m)")
                return 'LEFT'
            else:
                self.get_logger().info(f"↱ Decisión: DERECHA (empate, {right:.2f}m)")
                return 'RIGHT'

    def fallback_obstacle_decision(self):
        """Decisión de emergencia si el escaneo falla"""
        self.get_logger().warn("Escaneo falló - usando sensores actuales como fallback")
        
        left = self.distance_left
        right = self.distance_right
        front = self.distance_front
        
        self.get_logger().info(f"Lecturas actuales → F={front:.2f}m, L={left:.2f}m, R={right:.2f}m")
        
        if left > right + 0.10:
            self.get_logger().info(f"↰ Fallback: IZQUIERDA ({left:.2f}m)")
            return 'LEFT'
        elif right > left + 0.10:
            self.get_logger().info(f"↱ Fallback: DERECHA ({right:.2f}m)")
            return 'RIGHT'
        else:
            # Si empate, usar el lado que tuvo mejor lectura histórica
            if left >= right:
                self.get_logger().info(f"↰ Fallback: IZQUIERDA (empate)")
                return 'LEFT'
            else:
                self.get_logger().info(f"↱ Fallback: DERECHA (empate)")
                return 'RIGHT'

    def check_if_stuck(self):
        """Detección robusta - NO detectar atasco durante APPROACHING"""
        if self.nav_state not in ['SEARCHING']:
            self.odom_stuck_start = None
            return False
        
        if abs(self.last_speed_command) < 0.15:
            self.odom_stuck_start = None
            return False
        
        if not self.odom_initialized or self.odom_distance == 0.0:
            return False
        
        if self.odom_init_time is not None:
            if time.time() - self.odom_init_time < self.odom_warmup_time:
                return False
        
        if time.time() - self.last_odom_update_time > self.odom_timeout:
            if not self.motor_failure_detected:
                self.get_logger().error("FALLO DE MOTORES: Odometría no actualiza")
                self.motor_failure_detected = True
                self.motor_recovery_attempts += 1
                
                if self.motor_recovery_attempts >= self.max_motor_recovery_attempts:
                    self.get_logger().error("Fallo crítico de motores - requiere reinicio")
                    return False
            
            return False
        
        odom_change = abs(self.odom_distance - self.last_odom_check)
        
        if odom_change > self.stuck_threshold:
            self.odom_stuck_start = None
            self.last_odom_check = self.odom_distance
            return False
        
        if self.odom_stuck_start is None:
            self.odom_stuck_start = time.time()
            self.last_odom_check = self.odom_distance
            return False
        
        if time.time() - self.odom_stuck_start >= self.stuck_timeout:
            self.get_logger().warn("ATASCO DETECTADO")
            return True
        
        return False

    def publish_cmd_vel(self, linear, angular):
        """Publicar con verificación"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        
        self.last_speed_command = linear

    def control_loop(self):
        """🔧 Máquina de estados con escaneo robusto"""
        now = time.time()
        
        # Si hay fallo de motores, intentar recuperar
        if self.motor_failure_detected and self.motor_recovery_attempts < self.max_motor_recovery_attempts:
            self.get_logger().warn("Intentando recuperar motores...")
            self.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.5)
            self.last_odom_update_time = time.time()
        
        # PRIORIDAD: Obstáculos inmediatos
        if (self.nav_state in ['SEARCHING', 'ALIGNING', 'APPROACHING', 'VISUAL_SEARCH'] 
            and self.distance_front < self.obstacle_threshold):
            
            self.get_logger().warn(f"OBSTÁCULO a {self.distance_front:.2f}m")
            self.nav_state = 'OBSTACLE_STOP'
            self.vision_paused = True
            self.centered_time = None
            self.publish_cmd_vel(0.0, 0.0)
            return
        
        # WAITING
        if self.nav_state == 'WAITING':
            self.publish_cmd_vel(0.0, 0.0)
            self.set_servo_angle(90)
        
        # VERIFICAR ATASCO
        elif self.check_if_stuck():
            self.last_stuck_position = self.odom_distance
            self.stuck_recovery_attempts += 1
            self.centered_time = None
            
            if self.stuck_recovery_attempts > self.max_recovery_attempts:
                self.get_logger().error("Múltiples atascos - requiere intervención")
                self.nav_state = 'WAITING'
                self.vision_paused = True
                self.publish_cmd_vel(0.0, 0.0)
            else:
                self.nav_state = 'STUCK_REVERSE'
                self.reverse_counter = int(self.reverse_duration / 0.1)
                self.vision_paused = True
                self.publish_cmd_vel(self.reverse_speed, 0.0)
        
        # STUCK_REVERSE
        elif self.nav_state == 'STUCK_REVERSE':
            self.publish_cmd_vel(self.reverse_speed, 0.0)
            self.reverse_counter -= 1
            
            if self.reverse_counter <= 0:
                self.get_logger().info("Retroceso completo")
                self.nav_state = 'OBSTACLE_STOP'
                self.publish_cmd_vel(0.0, 0.0)
        
        # OBSTACLE_STOP
        elif self.nav_state == 'OBSTACLE_STOP':
            self.publish_cmd_vel(0.0, 0.0)
            
            if not self.waiting_scan:
                self.request_obstacle_scan()
                self.nav_state = 'OBSTACLE_SCAN'
        
        # 🔧 OBSTACLE_SCAN - MEJORADO CON TIMEOUT Y FALLBACK
        elif self.nav_state == 'OBSTACLE_SCAN':
            self.publish_cmd_vel(0.0, 0.0)
            
            elapsed = now - self.scan_request_time
            
            # Verificar si completó
            if self.check_scan_complete():
                self.get_logger().info(f"Escaneo completo en {elapsed:.1f}s")
                self.waiting_scan = False
                self.servo_locked = False
                self.scan_retry_count = 0
                
                decision = self.analyze_obstacle_scan()
                self.nav_state = 'TURNING'
                self.turn_direction = decision
                self.turn_counter = int(self.turn_duration_45 / 0.1)
            
            # Timeout alcanzado
            elif elapsed > self.scan_timeout:
                self.get_logger().error(f"Timeout escaneo ({elapsed:.1f}s) - datos: {self.scan_data_count}/3")
                
                # ¿Podemos reintentar?
                if self.scan_retry_count < self.max_scan_retries:
                    self.scan_retry_count += 1
                    self.get_logger().warn(f"Reintentando escaneo ({self.scan_retry_count}/{self.max_scan_retries})...")
                    
                    # Resetear y reintentar
                    self.waiting_scan = False
                    self.servo_locked = False
                    self.scan_data_count = 0
                    time.sleep(0.5)  # Pausa breve
                    
                    self.request_obstacle_scan()
                else:
                    # Ya reintentamos suficiente - usar fallback
                    self.get_logger().error(f"Escaneo falló después de {self.max_scan_retries} reintentos")
                    self.waiting_scan = False
                    self.servo_locked = False
                    self.scan_retry_count = 0
                    
                    # Usar lecturas actuales de sensores
                    self.turn_direction = self.fallback_obstacle_decision()
                    self.nav_state = 'TURNING'
                    self.turn_counter = int(self.turn_duration_45 / 0.1)
            
            # Todavía esperando...
            else:
                remaining = self.scan_timeout - elapsed
                if int(elapsed * 10) % 5 == 0:  # Log cada 0.5s
                    self.get_logger().info(f"Esperando escaneo... {remaining:.1f}s restantes (datos: {self.scan_data_count}/3)")
        
        # TURNING
        elif self.nav_state == 'TURNING':
            turn_speed = self.turn_speed_45 if self.turn_direction == 'LEFT' else -self.turn_speed_45
            self.publish_cmd_vel(0.0, turn_speed)
            
            self.turn_counter -= 1
            
            if self.turn_counter <= 0:
                self.get_logger().info("Giro completado")
                self.nav_state = 'SEARCHING'
                self.vision_paused = False
                self.last_search_time = now - self.search_interval
                self.turn_direction = None
                self.odom_stuck_start = None
                self.last_odom_check = self.odom_distance
        
        # SEARCHING
        elif self.nav_state == 'SEARCHING':
            if self.color_detectado and self.color_area > self.area_detected_threshold:
                self.nav_state = 'ALIGNING'
                self.centered_time = None
                self.get_logger().info(f"{self.color_objetivo} detectado! Área={self.color_area}")
            elif now - self.last_search_time >= self.search_interval:
                self.nav_state = 'VISUAL_SEARCH'
                self.scan_angle = 90
                self.scan_direction = 1
                self.set_servo_angle(90)
                self.publish_cmd_vel(0.0, 0.0)
                self.get_logger().info("Barrido visual")
            else:
                self.publish_cmd_vel(self.cruise_speed, 0.0)
        
        # VISUAL_SEARCH
        elif self.nav_state == 'VISUAL_SEARCH':
            self.publish_cmd_vel(0.0, 0.0)
            
            if self.color_detectado and self.color_area > self.area_detected_threshold:
                self.nav_state = 'ALIGNING'
                self.centered_time = None
                self.last_search_time = now
                self.set_servo_angle(90)
                self.get_logger().info(f"{self.color_objetivo} encontrado!")
            else:
                self.scan_angle += self.scan_direction * self.scan_speed
                
                if self.scan_angle >= 180:
                    self.scan_angle = 180
                    self.scan_direction = -1
                elif self.scan_angle <= 0:
                    self.scan_angle = 0
                    self.nav_state = 'SEARCHING'
                    self.last_search_time = now
                    self.set_servo_angle(90)
                
                self.set_servo_angle(self.scan_angle)
                time.sleep(self.scan_delay)
        
        # ALIGNING
        elif self.nav_state == 'ALIGNING':
            if now - self.ultimo_dato_vision > self.vision_timeout:
                self.nav_state = 'VISUAL_SEARCH'
                self.scan_angle = 90
                self.scan_direction = 1
                self.centered_time = None
                self.publish_cmd_vel(0.0, 0.0)
                self.get_logger().info(" Color perdido")
            elif self.color_posicion_x == 0:
                if self.centered_time is None:
                    self.centered_time = now
                    self.area_when_centered = self.color_area
                    self.get_logger().info(f"Color centrado (área={self.color_area}) - verificando estabilidad...")
                    self.publish_cmd_vel(0.0, 0.0)
                elif now - self.centered_time >= self.required_centered_duration:
                    self.nav_state = 'APPROACHING'
                    self.set_servo_angle(90)
                    self.publish_cmd_vel(0.0, 0.0)
                    self.get_logger().info(f"{self.color_objetivo} CENTRADO Y ESTABLE - acercándose")
                else:
                    self.publish_cmd_vel(0.0, 0.0)
            elif self.color_posicion_x == -1:
                self.centered_time = None
                self.publish_cmd_vel(self.approach_speed * 0.6, self.turn_speed_align * 0.8)
            else:
                self.centered_time = None
                self.publish_cmd_vel(self.approach_speed * 0.6, -self.turn_speed_align * 0.8)
        
        # APPROACHING
        elif self.nav_state == 'APPROACHING':
            if self.color_area > 0:
                estimated_dist_cm = (2290 * 40) / max(self.color_area, 100)
                estimated_dist_cm = min(estimated_dist_cm, 100)
            else:
                estimated_dist_cm = 100
            
            if time.time() - self.last_area_log > 0.8:
                self.get_logger().info(
                    f"Área: {self.color_area}/{self.area_reached_threshold} "
                    f"(~{estimated_dist_cm:.0f}cm) | "
                    f"Pos: {'Centro' if self.color_posicion_x == 0 else ('Izq' if self.color_posicion_x == -1 else 'Der')}"
                )
                self.last_area_log = time.time()
            
            if self.color_area >= self.area_reached_threshold:
                if not hasattr(self, '_approach_history'):
                    self._approach_history = []
                
                self._approach_history.append(self.color_area)
                if len(self._approach_history) > 10:
                    self._approach_history.pop(0)
                
                if len(self._approach_history) >= 3:
                    area_growth = self._approach_history[-1] - self._approach_history[0]
                    
                    if self.color_objetivo == "AMARILLO" and area_growth < 5000:
                        self.get_logger().warn(f" Amarillo sin progresión clara ({area_growth}) - continuando acercamiento")
                        if self.color_posicion_x == 0:
                            self.publish_cmd_vel(self.approach_speed * 0.5, 0.0)
                        else:
                            correction = self.turn_speed_align * 0.5
                            angular = correction if self.color_posicion_x == -1 else -correction
                            self.publish_cmd_vel(self.approach_speed * 0.4, angular)
                        return
                
                if self.color_posicion_x == 0:
                    if self.centered_time is None:
                        self.centered_time = now
                        self.get_logger().info(f"Área OK ({self.color_area}) - verificando centrado...")
                        self.publish_cmd_vel(0.0, 0.0)
                    elif now - self.centered_time >= self.required_centered_duration:
                        self.get_logger().info(f"LLEGADA: {self.color_objetivo} (área={self.color_area})")
                        
                        if self.color_objetivo == "AMARILLO":
                            self._yellow_close_confirmed = True
                        
                        try:
                            requests.get(
                                f"http://{self.ESP32_IP}/color",
                                params={"c": self.color_objetivo},
                                timeout=2
                            )
                        except:
                            pass
                        
                        self.nav_state = 'WAITING'
                        self.vision_paused = True
                        self.stuck_recovery_attempts = 0
                        self.centered_time = None
                        if hasattr(self, '_approach_history'):
                            delattr(self, '_approach_history')
                        self.publish_cmd_vel(0.0, 0.0)
                    else:
                        self.publish_cmd_vel(0.0, 0.0)
                else:
                    self.get_logger().warn(f"Área OK pero descentrado - ajustando...")
                    self.centered_time = None
                    correction = self.turn_speed_align * 0.5
                    angular = correction if self.color_posicion_x == -1 else -correction
                    self.publish_cmd_vel(0.0, angular)
            
            elif now - self.ultimo_dato_vision > self.vision_timeout:
                self.get_logger().warn(" Color perdido")
                self.nav_state = 'VISUAL_SEARCH'
                self.scan_angle = 90
                self.scan_direction = 1
                self.centered_time = None
                self.publish_cmd_vel(0.0, 0.0)
            
            elif self.color_posicion_x != 0:
                self.centered_time = None
                correction = self.turn_speed_align * 0.7
                angular = correction if self.color_posicion_x == -1 else -correction
                if self.color_area < self.area_reached_threshold * 0.2:
                    self.publish_cmd_vel(self.approach_speed, angular)
                elif self.color_area < self.area_reached_threshold * 0.6:
                    self.publish_cmd_vel(self.approach_speed * 0.8, angular)
                else:
                    self.publish_cmd_vel(self.approach_speed * 0.6, angular)
            else:
                if self.color_area < self.area_reached_threshold * 0.3:
                    self.publish_cmd_vel(self.approach_speed, 0.0)
                elif self.color_area < self.area_reached_threshold * 0.7:
                    self.publish_cmd_vel(self.approach_speed * 0.8, 0.0)
                else:
                    self.publish_cmd_vel(self.approach_speed * 0.5, 0.0)

    def destroy_node(self):
        """Limpieza"""
        self.vision_running = False
        if self.vision_thread:
            self.vision_thread.join(timeout=2)
        
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo robot...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()