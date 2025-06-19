import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from math import radians, cos, sin, sqrt, atan2, degrees
import gpiod
import time

CHIP = 'gpiochip4'
IN1 = 13
IN2 = 6
IN3 = 19
IN4 = 26
ENA = 17
ENB = 27
TRIG = 24
ECHO = 23
ENGEL_MESAFE = 5  # cm

class SoftPWM:
    def __init__(self, pin, freq=1000):
        self.line = gpiod.Chip(CHIP).get_line(pin)
        self.line.request(consumer="PWM", type=gpiod.LINE_REQ_DIR_OUT)
        self.freq = freq
        self.duty_cycle = 0.5
        self.running = False

    def start(self):
        self.running = True
        from threading import Thread
        Thread(target=self._run, daemon=True).start()

    def _run(self):
        period = 1 / self.freq
        while self.running:
            self.line.set_value(1)
            time.sleep(self.duty_cycle * period)
            self.line.set_value(0)
            time.sleep((1 - self.duty_cycle) * period)

    def set_duty_cycle(self, dc):
        self.duty_cycle = max(0.0, min(1.0, dc))

    def stop(self):
        self.running = False
        self.line.set_value(0)
        self.line.release()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot1_controller')
        self.current_position = None
        self.target_position = None

        self.create_subscription(NavSatFix, '/robot1/gps', self.gps_callback, 10)
        self.create_subscription(String, '/robot1/task', self.task_callback, 10)

        chip = gpiod.Chip(CHIP)
        self.in1 = chip.get_line(IN1)
        self.in2 = chip.get_line(IN2)
        self.in3 = chip.get_line(IN3)
        self.in4 = chip.get_line(IN4)
        self.trig = chip.get_line(TRIG)
        self.echo = chip.get_line(ECHO)

        for line in [self.in1, self.in2, self.in3, self.in4, self.trig]:
            line.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.echo.request(consumer="ECHO", type=gpiod.LINE_REQ_DIR_IN)

        self.ena_pwm = SoftPWM(ENA)
        self.enb_pwm = SoftPWM(ENB)
        self.ena_pwm.start()
        self.enb_pwm.start()
        self.ena_pwm.set_duty_cycle(0.5)
        self.enb_pwm.set_duty_cycle(0.5)

        self.create_timer(1.0, self.navigate_to_target)

    def gps_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude)
        self.get_logger().info(f"Konum: {self.current_position}")

    def task_callback(self, msg):
        try:
            lat, lon = map(float, msg.data.strip().split(','))
            self.target_position = (lat, lon)
            self.get_logger().info(f"Görev: {self.target_position}")
        except:
            self.get_logger().warn("Görev verisi hatalı")

    def haversine(self, coord1, coord2):
        R = 6371e3
        φ1, φ2 = radians(coord1[0]), radians(coord2[0])
        Δφ = radians(coord2[0] - coord1[0])
        Δλ = radians(coord2[1] - coord1[1])
        a = sin(Δφ / 2) ** 2 + cos(φ1) * cos(φ2) * sin(Δλ / 2) ** 2
        return R * 2 * atan2(sqrt(a), sqrt(1 - a))

    def calculate_bearing(start, end):
        lat1, lon1 = map(radians, start)
        lat2, lon2 = map(radians, end)
        d_lon = lon2 - lon1
    
        x = sin(d_lon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(d_lon)
    
        bearing = atan2(x, y)
        bearing = (degrees(bearing) + 360) % 360
        return bearing

    def ileri(self):
        self.in1.set_value(1)
        self.in2.set_value(0)
        self.in3.set_value(1)
        self.in4.set_value(0)

    def dur(self):
        self.in1.set_value(0)
        self.in2.set_value(0)
        self.in3.set_value(0)
        self.in4.set_value(0)

    def saga_don(self):
        self.in1.set_value(1)
        self.in2.set_value(0)
        self.in3.set_value(0)
        self.in4.set_value(1)

    def get_distance(self):
        self.trig.set_value(0)
        time.sleep(0.000002)
        self.trig.set_value(1)
        time.sleep(0.00001)
        self.trig.set_value(0)

        pulse_start = time.time()
        timeout = time.time() + 0.04
        while self.echo.get_value() == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return 999

        pulse_end = time.time()
        while self.echo.get_value() == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return 999

        duration = pulse_end - pulse_start
        return duration * 17150

    def navigate_to_target(self):
        if not self.current_position or not self.target_position:
            return
    
        distance = self.haversine(self.current_position, self.target_position)
        self.get_logger().info(f"Hedefe uzaklık: {distance:.1f} metre")
    
        if distance < 3.0:
            self.get_logger().info("Hedefe ulaşıldı! Araç duruyor.")
            self.dur()
            return
    
        # Engel kontrolü
        mesafe = self.get_distance()
        if mesafe < ENGEL_MESAFE:
            self.get_logger().info("Engel tespit edildi, sağa kaçılıyor.")
            self.saga_don()
            time.sleep(1.0)
            self.dur()
            return
    
        # Bearing hesapla
        bearing_to_target = calculate_bearing(self.current_position, self.target_position)
    
        if not hasattr(self, 'last_position'):
            self.last_position = self.current_position
            self.ileri()
            return
    
        # Yaklaşık araç yönünü hesapla
        heading_now = calculate_bearing(self.last_position, self.current_position)
        delta = (bearing_to_target - heading_now + 360) % 360
        self.last_position = self.current_position
    
        self.get_logger().info(f"Hedef yönü: {bearing_to_target:.1f}°, Araç yönü: {heading_now:.1f}°, Sapma: {delta:.1f}°")
    
        # Küçük sapmalar için düz git, büyükse düzelt
        if delta < 15 or delta > 345:
            self.get_logger().info("Yön doğru, ileri gidiliyor.")
            self.ileri()
        elif delta < 180:
            self.get_logger().info("Hedef sağda, sağa dönülüyor.")
            self.saga_don()
            time.sleep(0.5)
            self.dur()
        else:
            self.get_logger().info("Hedef solda, sola dönülüyor.")
            self.sola_don()
            time.sleep(0.5)
            self.dur()


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.dur()
    node.ena_pwm.stop()
    node.enb_pwm.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
