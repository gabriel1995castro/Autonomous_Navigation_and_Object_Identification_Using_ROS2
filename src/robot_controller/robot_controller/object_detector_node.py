#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import csv

import numpy as np
from scipy.optimize import leastsq
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

class SaveInfoCSV:
    
    def __init__(self, file_path='Lista_Objetos.csv'):
        self.file_path = file_path

    def save_object(self, tipo, x, y):

        objetos = self.load_objects()
        if (tipo, x, y) not in objetos:
            with open(self.file_path, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([tipo, x, y])

    def load_objects(self):

        objetos = []
        try:
            with open(self.file_path, mode="r") as file:
                reader = csv.reader(file)
                for row in reader:
                    objetos.append((row[0], float(row[1]), float(row[2])))
        except FileNotFoundError:
            pass  
        return objetos

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('scan_detect')

        self.csv_storage = SaveInfoCSV()
        self.lista_objetos = self.csv_storage.load_objects()

        self._init_variables()
        self._init_subs()

        # Configuração do timer
        self.timer = self.create_timer(0.3, self.loop)

        # Configuração do gráfico
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        plt.show(block=False)
        self.get_logger().info("Node started")

    def _init_variables(self):

        self.robo_pose = None
        self.scan_data = None
        self.objetivo_position = Pose2D()
        self.objet_list = []

    def _init_subs(self):

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

    def scan_callback(self, scan_data):

        self.scan_data = scan_data

    def pose_callback(self, pose_data):

        self.robo_pose = pose_data.pose.pose

    def loop(self):

        if self.robo_pose is None or self.scan_data is None:
            return

        objects = self.detect_objects()

        for obj in objects:
            if obj not in self.objet_list:
                self.objet_list.append(obj)
                self.csv_storage.save_object(obj[0], obj[1], obj[2])

        self.get_logger().info(f"Objetos detectados: {self.objet_list}")

        self.ax.clear()
        for obj in objects:
            self.ax.scatter(obj[1], obj[2], marker='o', color='g')
            self.ax.text(obj[1], obj[2], obj[0], fontsize=12, color='black')

        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_aspect('equal')
        plt.draw()
        plt.pause(0.001)

    def detect_objects(self):

        if self.scan_data is None or self.robo_pose is None:
            return []

        points = self.convert_to_cartesian()
        clusters = self.cluster_objects(points)
        return self.classify_objects(clusters)

    def convert_to_cartesian(self):

        angle = self.scan_data.angle_min
        points = []

        for r in self.scan_data.ranges:
            if self.scan_data.range_min < r < self.scan_data.range_max:
                x = r * np.cos(angle) + self.robo_pose.position.x
                y = r * np.sin(angle) + self.robo_pose.position.y
                points.append([x, y])
            angle += self.scan_data.angle_increment

        return np.array(points)

    def cluster_objects(self, points):

        clustering = DBSCAN(eps=0.3, min_samples=5).fit(points)
        labels = clustering.labels_

        clusters = []
        for label in np.unique(labels):
            if label != -1:
                clusters.append(points[labels == label])

        return clusters

    def classify_objects(self, clusters):

        detect_obj = []

        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)

            if self.is_sphere(cluster):
                tipo = "esfera"
            elif self.is_cylinder(cluster):
                tipo = "cilindro"
            else:
                tipo = "caixa"

            detect_obj.append((tipo, centroid[0], centroid[1]))

        return detect_obj

    def is_sphere(self, cluster):

        def residuals(params, points):
            x_c, y_c, r = params
            return np.sqrt((points[:, 0] - x_c) ** 2 + (points[:, 1] - y_c) ** 2) - r

        x0 = np.mean(cluster, axis=0)
        r0 = np.std(cluster)
        params, _ = leastsq(residuals, [x0[0], x0[1], r0], args=(cluster,))

        erro_medio = np.mean(np.abs(residuals(params, cluster)))
        return erro_medio < 0.1

    def is_cylinder(self, cluster):

        def residuals(params, points):
            a, b, c = params
            return np.abs(a * points[:, 0] + b * points[:, 1] + c) / np.sqrt(a ** 2 + b ** 2)

        x0 = [1, -1, 0]
        params, _ = leastsq(residuals, x0, args=(cluster,))
        erro_medio = np.mean(np.abs(residuals(params, cluster)))
        return erro_medio < 0.1

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()