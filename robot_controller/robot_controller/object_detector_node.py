import rclpy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import tf_transformations as tf
import json
import os
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool 
from sklearn.cluster import DBSCAN
from scipy import stats
from scipy.spatial import ConvexHull
from skimage.measure import CircleModel, ransac

# Configura o backend do Matplotlib para exibição gráfica
matplotlib.use('TkAgg')

class ObjectRecognition(Node):
    def __init__(self):
        super().__init__('object_recognition')
        
        #Declara as funções usadas para receber variáveis, pusblishers e subscribers.
        self.setup_variables()
        self.setup_subscribers()
        self.setup_publishers()
        
        # Define timers para execução periódica de funções
        self.timer = self.create_timer(0.4, self.run_detection)
        self.end_timer = self.create_timer(5.0, self.check_stagnation)
        self.start_time = time.time()
        self.initialize_plot()
        
        # Caminho relativo para salvar o arquivo json com os objetos encontrados.
        self.json_file_path = os.path.abspath("detected_objects.json")
        self.load_detected_objects()
        self.get_logger().info("Sistema de reconhecimento de objetos iniciado.")

    def setup_variables(self):
        # Inicializa variáveis
        self.robot_position = None
        self.lidar_data = None
        self.object_classification = 1
        self.target_position = Pose2D()
        self.target_position.theta = 0.0
        self.last_object_count = 0 
        self.last_update_time = time.time()  
        self.detected_objects = []
    
    def setup_subscribers(self):
        # Configura assinaturas de tópicos do ROS2 para o LIdar, Odometria e um tópico publicado pelo nó de detecção
        # de objetos para se utilizado como um critério de parada.
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/stop_flag', self.stop_flag_callback, 10)
       

    def setup_publishers(self):
        # Inicializa os tópicos resposánveis por publicar a posição do objeto e a flag de parada usada na ausência
        # de detecções de novos objetos.
        self.object_position_publisher = self.create_publisher(Pose2D, '/detected_object_position', 10)
        self.stop_flag_publisher = self.create_publisher(Bool, '/stop_flag', 10)  
    
    def stop_flag_callback(self, msg):
        # Usado para finalizar a execução dos nós.
        if msg.data:  
            self.get_logger().info("Flag de parada recebida. Encerrando o programa.")
            rclpy.shutdown()
   
    def lidar_callback(self, msg):
        # Recebe os dados provindos do LiDAR
        self.lidar_data = msg
    
    def odom_callback(self, msg):
        # Recebe os dados da odometria do robô
        self.robot_position = msg.pose.pose
    
    
    def initialize_plot(self):
        # Inicializa o gráfico usado para exibir a detecção de objetos durante a execução do nó.
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        plt.show(block=False)
    
    def run_detection(self):
        # Função principal usada para a detecção.
        if self.robot_position is None or self.lidar_data is None:
            self.get_logger().warn("Dados insuficientes para detecção.") #Caso o LIdar esteja indisponível exibe um alerta no terminal.
            return
        
        self.ax.clear() #Limpa o  gráfico a cada iteração para exibição de novos dados.
        scan_data = self.lidar_data
        posicao_atual = [self.robot_position.position.x, self.robot_position.position.y]
        orientacao = self.robot_position.orientation
        
        # Converte dados do LiDAR para coordenadas cartesianas
        coordenadas_obstaculos, _ = self.transform_to_cartesian(scan_data, posicao_atual, orientacao)
        _, clusters = self.identify_obstacles(coordenadas_obstaculos, self.object_classification)
        
        self.ax.scatter(coordenadas_obstaculos[:, 0], coordenadas_obstaculos[:, 1], marker='.', color='k')
        centroides = []
        #Percorre os dados dos clusters e chama a função para cálculo do centroíde dos objetos detectados.
        for cluster in clusters:
            centroide = self.compute_centroid(cluster)
            if centroide is not None:
                width, height = self.compute_dimensions(cluster) #Chama a função para obter a medição do tamanho do objeto encontrado.
                distance = np.linalg.norm(np.array(centroide) - np.array(posicao_atual)) #Avalia se o objeto está a um distância correta para ser adicionado a lista.
                if 0.2 < distance < 0.8:
                    object_type = self.classify_object(cluster, distance)#Executa a classificação do tipo de objeto.
                    color = 'r' if object_type == 'Caixa' else ('g' if object_type == 'Esfera' else 'y')
                    centroides.append({"Tipo": object_type, "Localizacao": centroide, "Largura": width, "Altura": height})
                    if object_type != "Desconhecido":#Elimina os objetos detectados fora do espaço de análise escolhido.
                        self.ax.text(centroide[0], centroide[1], f"{object_type}\n{width:.2f} x {height:.2f} m", color="black")
                    self.ax.scatter(cluster[:, 0], cluster[:, 1], marker='.', color=color)
                    self.ax.scatter(centroide[0], centroide[1], marker='.', color=color)
                    self.get_logger().info(f"Objeto detectado: {object_type}, Tamanho: {width:.2f} x {height:.2f} m")
        
        self.update_detected_objects(centroides)
        self.save_detected_objects()
        # Atualiza a visualização
        self.ax.scatter(posicao_atual[0], posicao_atual[1], marker='H', color='b')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_aspect('equal')
        plt.draw()
        plt.pause(0.001)
    
    #Executa o cálculo do centroíde do cluster analisado.
    def compute_centroid(self, points):
        if len(points) < 3: #Avalia se existe uma quantidade superior a três pontos, elimina ruídos.
            return None
        x_coords = points[:, 0]
        y_coords = points[:, 1]
        centroid = [np.mean(x_coords), np.mean(y_coords)] 
        return centroid
    
    def compute_dimensions(self, points):
        width = np.ptp(points[:, 0])  # Diferença entre máximo e mínimo de x
        height = np.ptp(points[:, 1])  # Diferença entre máximo e mínimo de y
        return width, height
    
    #Converte as coordenadas para serem do tipo 2D.
    def transform_to_cartesian(self, scan_data, robot_position, robot_orientation):
        angle = scan_data.angle_min
        x, y = [], []
        rotation_matrix = tf.quaternion_matrix([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])
        
        for r in scan_data.ranges:
            if scan_data.range_min <= r <= scan_data.range_max:
                local_x = r * np.cos(angle)
                local_y = r * np.sin(angle)
                point_world = np.dot(rotation_matrix, [local_x, local_y, 0, 1])[:3]
                x.append(point_world[0] + robot_position[0])
                y.append(point_world[1] + robot_position[1])
            angle += scan_data.angle_increment
        
        return np.array([x, y]).T, None
    
    #Faz a indentificação dos objetos.
    def identify_obstacles(self, values, scale=0):
        max_size, min_size = (40, 5) if scale == 1 else (80, 10)
        clusters = []
        #Utiliza a função DBSCAN considerando uma distância máxima de 3 entre pontos e no minímo 3 pontos para fomar um cluster.
        labels = DBSCAN(eps=0.3, min_samples=3).fit(values).labels_ 
        #Filtra os ruídos (-1).
        for i in set(labels):
            if i == -1:
                continue
            cluster = values[labels == i]
            if min_size < len(cluster) < max_size:
                _, _, r_value, _, _ = stats.linregress(cluster[:, 0], cluster[:, 1])
                if 0.08 < r_value**2 < 1:
                    clusters.append(cluster)
        
        return None, clusters
    
    
    #Ajusta uma circunferência usando RANSAC para ignorar pontos fora do modelo.
    def fit_circle(self, points):

        model = CircleModel()
        try:
            model, inliers = ransac(points, CircleModel, min_samples=5, residual_threshold=0.03, max_trials=200)
            xc, yc, r = model.params
            error = np.mean(np.abs(np.sqrt((points[:, 0] - xc)**2 + (points[:, 1] - yc)**2) - r))

           
            theta = np.linspace(0, 2*np.pi, 100)
            x_fit = xc + r * np.cos(theta)
            y_fit = yc + r * np.sin(theta)
            #self.ax.plot(x_fit, y_fit, 'm--', label="Ajuste de círculo")

            return xc, yc, r, error

        except ValueError:
            return None, None, None, float('inf')  

    # Classifica o objeto detectado com base na forma e distância
    def classify_object(self, points, distance):
       #Ignora os objetos longe dotando eles diretamente com a flag ("Desconhecido").
        if distance > 1.5:
            return 'Desconhecido'
        
        xc, yc, r, error = self.fit_circle(points)
        if xc is None:
            return 'Caixa'  # Se não conseguiu ajustar um círculo, assume que é uma caixa

        # Calcula a área da casca convexa comparada à área do círculo ajustado.
        hull = ConvexHull(points)
        area_hull = hull.volume
        area_circle = np.pi * (r ** 2)
        circularity = area_hull / area_circle

        # Se a forma for muito alongada (circularity < 0.7), provavelmente é uma caixa
        if error < 0.05 and 0.3 < r < 1.5 and circularity > 0.95:
            return 'Esfera'
        
        return 'Caixa'

    def save_detected_objects(self):
        self.get_logger().info("Tentando salvar os objetos detectados...")
        self.get_logger().info(f"Lista de objetos: {self.detected_objects}")
        with open(self.json_file_path, 'w') as file:
            json.dump(self.detected_objects, file, indent=4)
        self.get_logger().info(f"Objetos salvos em {self.json_file_path}")

    
    def load_detected_objects(self):
        try:
            with open(self.json_file_path, 'r') as file:
                self.detected_objects = json.load(file)
        except FileNotFoundError:
            self.detected_objects = []
            
    #Implementa uma função de verificação para armazenamento de objetos.    
    def update_detected_objects(self, new_points, threshold=0.8):
        
        for obj in new_points:
           #Só serão armazenados objetos que não possuem a flag Desconhecido
           #Objetos com centroides com valores inferiores a 0.8 de distância de objetos presentes na lista também são ignorados
            if obj["Tipo"] != "Desconhecido" and not any(np.linalg.norm(np.array(obj["Localizacao"]) - np.array(existing["Localizacao"])) < threshold for existing in self.detected_objects):
               
                self.detected_objects.append(obj)
       
        current_object_count = len(self.detected_objects)
        #Atualiza a quantidade objetos já detectados presentes na lista.
        if current_object_count > self.last_object_count:
            self.last_object_count = current_object_count
            self.last_update_time = time.time() 
    
    #Verifica se  a lista de objetos foi atualizada.
    def check_stagnation(self):
        #Se nenhum objeto foi detectado durante 2 minutos dispara uma flag que encerra o funcionamento do nó.
        if time.time() - self.last_update_time > 120: 
            self.get_logger().info("Nenhum novo objeto detectado em 2 minutos. Encerrando.")
            self.publish_stop_flag(True)               
    
    #Publica a flag de parada.
    def publish_stop_flag(self, value: bool):
       
        msg = Bool()
        msg.data = value
        self.stop_flag_publisher.publish(msg)
        
    #Gera e exibe um relatório de execução quando o nó for encerrado.
    def generate_report(self):
        
        total_time = time.time() - self.start_time
        num_objects = len(self.detected_objects)
        last_update = time.time() - self.last_update_time

        report = f"""
        🔹 **Relatório de Execução**
        ---------------------------------
         Tempo total de execução: {total_time:.2f} segundos
         Objetos detectados: {num_objects}
         Última atualização de detecção: {last_update:.2f} segundos atrás
         Localização final do robô: {self.get_final_position()}
        ---------------------------------
        """

        self.get_logger().info(report)
        self.save_report(report)  

    #Retorna a posição final do robô.
    def get_final_position(self):
        
        if self.robot_position:
            x = self.robot_position.position.x
            y = self.robot_position.position.y
            return f"X: {x:.2f}, Y: {y:.2f}"
        return "Desconhecida"

    def save_report(self, report):
        #Salva o relatório de execução em um arquivo de log.
        report_path = os.path.join(os.path.dirname(__file__), "execution_report.txt")
        with open(report_path, 'w') as file:
            file.write(report)
        self.get_logger().info(f"Relatório salvo em {report_path}")

    def destroy_node(self):
        self.generate_report()
        super().destroy_node()
        
def main(args=None):
    # Inicializa o nó do ROS2
    rclpy.init(args=args)
    detector = ObjectRecognition()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info("Interrompido no terminal.")
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
