import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import math

def convert_to_degrees(raw, direction):
    if not raw or not direction:
        return float('nan')
    try:
        degrees = float(raw[:2])
        minutes = float(raw[2:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return round(decimal, 6)
    except:
        return float('nan')

def parse_GPRMC(sentence):
    parts = sentence.split(",")
    return {
        "type": "GPRMC",
        "valid": parts[2] == 'A',
        "latitude": convert_to_degrees(parts[3], parts[4]) if len(parts) > 5 else float('nan'),
        "longitude": convert_to_degrees(parts[5], parts[6]) if len(parts) > 7 else float('nan'),
    }

def parse_GPGGA(sentence):
    parts = sentence.split(",")
    return {
        "type": "GPGGA",
        "fix": parts[6] != '0' if len(parts) > 6 else False,
        "latitude": convert_to_degrees(parts[2], parts[3]) if len(parts) > 4 else float('nan'),
        "longitude": convert_to_degrees(parts[4], parts[5]) if len(parts) > 6 else float('nan'),
        "altitude": float(parts[9]) if len(parts) > 9 and parts[9] else float('nan'),
    }

class GPSReader(Node):
    def __init__(self):
        super().__init__('gps_reader')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps', 10)
        self.timer = self.create_timer(1.0, self.read_gps_data)  # 1 Hz
        self.port = '/dev/ttyUSB0'  # RP5'e USB ile bağlı GPS
        self.baudrate = 9600

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Bağlandı: {self.port}")
        except Exception as e:
            self.get_logger().error(f"Seri bağlantı hatası: {e}")
            self.ser = None

    def read_gps_data(self):
        if not self.ser:
            return
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("$GPRMC"):
                data = parse_GPRMC(line)
            elif line.startswith("$GPGGA"):
                data = parse_GPGGA(line)
            else:
                return

            msg = NavSatFix()
            msg.latitude = data.get("latitude", float('nan'))
            msg.longitude = data.get("longitude", float('nan'))
            msg.altitude = data.get("altitude", 0.0 if math.isnan(data.get("altitude", float('nan'))) else data["altitude"])
            msg.header.frame_id = "gps"
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(msg)
            self.get_logger().info(f"Yayınlandı: {msg.latitude}, {msg.longitude}")

        except Exception as e:
            self.get_logger().warn(f"GPS okuma hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
