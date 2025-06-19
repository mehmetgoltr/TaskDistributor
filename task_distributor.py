import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from math import radians, degrees, cos, sin, sqrt, atan2
import json
import math
import os

class TaskDistributor(Node):
    def __init__(self):
        super().__init__('task_distributor')
        self.get_logger().info("Task Distributor started.")

        self.robot1_pos = None
        self.robot2_pos = None

        self.task_pub_1 = self.create_publisher(String, '/robot1/task', 10)
        self.task_pub_2 = self.create_publisher(String, '/robot2/task', 10)

        self.create_subscription(NavSatFix, '/robot1/gps', self.robot1_callback, 10)
        self.create_subscription(NavSatFix, '/robot2/gps', self.robot2_callback, 10)

        self.load_boxes()

        self.tasks_assigned = False

        self.create_timer(5.0, self.assign_tasks)

    def load_boxes(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'config', 'kutular.json')
        with open(path, 'r') as f:
            self.boxes = json.load(f)
        self.get_logger().info(f"{len(self.boxes)} kutu yüklendi.")

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371e3
        φ1 = math.radians(lat1)
        φ2 = math.radians(lat2)
        Δφ = math.radians(lat2 - lat1)
        Δλ = math.radians(lon2 - lon1)

        a = math.sin(Δφ / 2)**2 + math.cos(φ1) * math.cos(φ2) * math.sin(Δλ / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def robot1_callback(self, msg):
        self.robot1_pos = (msg.latitude, msg.longitude)

    def robot2_callback(self, msg):
        self.robot2_pos = (msg.latitude, msg.longitude)

    def assign_tasks(self):
        if self.tasks_assigned or self.robot1_pos is None or self.robot2_pos is None:
            return

        distances = []
        for box in self.boxes:
            dist1 = self.haversine(self.robot1_pos[0], self.robot1_pos[1], box['lat'], box['lon'])
            dist2 = self.haversine(self.robot2_pos[0], self.robot2_pos[1], box['lat'], box['lon'])
            distances.append((box, dist1, dist2))

        assigned = []
        for _ in range(2):
            best = min(distances, key=lambda x: min(x[1], x[2]))
            distances.remove(best)
            assigned.append(best)

        for box, dist1, dist2 in assigned:
            task_msg = String()
            task_msg.data = f"{box['lat']}, {box['lon']}"
            if dist1 < dist2:
                self.task_pub_1.publish(task_msg)
                self.get_logger().info(f"Robot1 için görev: {box['id']} → {task_msg.data}")
            else:
                self.task_pub_2.publish(task_msg)
                self.get_logger().info(f"Robot2 için görev: {box['id']} → {task_msg.data}")

        self.tasks_assigned = True

def main(args=None):
    rclpy.init(args=args)
    node = TaskDistributor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
