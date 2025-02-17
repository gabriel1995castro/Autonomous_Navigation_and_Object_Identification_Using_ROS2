#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import time
import random
from nav_msgs.msg import Odometry

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Publicador de comandos de velocidade para o robô
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Assinatura dos sensores do robô
        self.subscription = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.stop_flag_subscriber = self.create_subscription(Bool, '/stop_flag', self.stop_flag_callback, 10)  
        
        # Parâmetros do sistema de navegação
        self.safe_distance = 0.7  # Distância mínima segura para evitar obstáculos
        self.last_change_time = time.time()  # Tempo da última mudança de direção
        self.change_interval = random.uniform(7, 20)  # Intervalo aleatório para mudança de direção
        
        # Parâmetros do mapeamento da área explorada
        self.zone_count = 0  # Contador de zonas visitadas
        self.zone_size = 4  # Tamanho da zona explorada
        self.map_origin = [0, 0]  # Origem do mapa
        self.resolution = 0.1  # Resolução do mapa
        self.map_data = None  # Dados do mapa
        self.map_width = 0  # Largura do mapa
        self.map_height = 0  # Altura do mapa
        self.visited_zones = {}  # Dicionário para armazenar zonas visitadas
        self.last_new_zone_time = time.time()  # Tempo da última zona nova visitada

    def create_zone(self, x_cord, y_cord, d_section):
        #Cria uma zona quadrada com tamanho definido por d_section.
        coord = []
        for x in range(x_cord - d_section, x_cord + d_section + 1):
            for y in range(y_cord - d_section, y_cord + d_section + 1):
                coord.append((x, y))
        return coord
        
    def odom_callback(self, msg):
        #Atualiza a posição do robô e verifica se já visitou a zona.
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Converte coordenadas reais para a grade
        map_x = int((x - self.map_origin[0]) / self.resolution)
        map_y = int((y - self.map_origin[1]) / self.resolution)
        current_point = (map_x, map_y)

        # Define a zona baseada no tamanho especificado
        zone_x = (map_x // self.zone_size) * self.zone_size
        zone_y = (map_y // self.zone_size) * self.zone_size
        zone = (zone_x, zone_y)

        # Gera todas as coordenadas da zona
        zone_points = self.create_zone(zone_x, zone_y, self.zone_size)
        
        if zone not in self.visited_zones:
            # Nova zona detectada
            self.zone_count += 1
            self.visited_zones[zone] = zone_points
            self.get_logger().info(f"Zona {self.zone_count} criada | Coordenadas: {map_x}, {map_y}")
        else:
            # Zona já visitada
            self.get_logger().info(f"Já visitado - Pertence à Zona {self.zone_count}")
        
    def lidar_callback(self, msg):
        #Callback do sensor LiDAR para detectar obstáculos e controlar o movimento.
        current_time = time.time()
        
        # Verifica se o tempo de exploração foi suficiente e encerra
        if current_time - self.last_new_zone_time > 40 and self.declare_end():
            self.get_logger().info("Fim da exploração! Parando navegação.")
            self.publish_stop_flag(True)
            self.stop_robot()
            rclpy.shutdown()
            return
        
        # Se passou tempo suficiente, muda a direção
        if current_time - self.last_change_time > self.change_interval:
            self.turn()
            self.last_change_time = current_time
            return

        twist = Twist()
        cone_in_degree = 45  # Ângulo de detecção frontal
        num_ranges = int((len(msg.ranges) / 360) * cone_in_degree) // 2
        important_ranges = msg.ranges[:num_ranges] + msg.ranges[-num_ranges:]
        min_distance = min(important_ranges)  # Distância mínima detectada

        # Se houver um obstáculo próximo, gira o robô
        if min_distance < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.2
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.publisher.publish(twist)
        
        # Mapeia zonas com base nos dados do LiDAR
        zone_id = hash(tuple(np.round(msg.ranges, decimals=1)))
        if zone_id not in self.visited_zones:
            self.visited_zones[zone_id] = []
            self.last_new_zone_time = current_time  
        self.visited_zones[zone_id].append(current_time)

    def turn(self):
        #Gira o robô aleatoriamente para mudar de direção
        twist = Twist()
        turn_duration = random.uniform(2, 5)  # Tempo de rotação
        angular_velocity = 0.2  # Velocidade angular
        twist.angular.z = angular_velocity if random.choice([True, False]) else -angular_velocity

        self.publisher.publish(twist)
        time.sleep(turn_duration)

        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.change_interval = random.uniform(7, 20)  # Define novo intervalo de mudança
        
    def stop_navigation(self):
        #Para o movimento do robô
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def publish_stop_flag(self, value: bool):
        #Publica um sinal de parada para encerrar a exploração
        msg = Bool()
        msg.data = value
        self.flag_publisher.publish(msg)
        
    def stop_flag_callback(self, msg):
        #Para o robô e encerra a navegação quando recebe o sinal de parada
        if msg.data:  # Se a flag de parada for True
            self.get_logger().info("Flag de parada recebida. Encerrando a navegação.")
            self.stop_navigation()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
