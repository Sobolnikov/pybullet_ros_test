import os
import subprocess
import rospy
import rospkg
import pybullet as p
import pybullet_data
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# Класс для работы с PyBullet и ROS
class PyBulletRosServer:
    def __init__(self):
        
        self.GRAVITY = -9.8  # Задаем значение силы тяжести
        self.robot_id = None  # Идентификатор робота в симуляции
        self.wheel_radius = 0.075  # Радиус колес
        self.wheel_distance = 0.45  # Расстояние между колесами
        self.joint_state_publisher = None  # Публикация состояния шарниров
        self.joint_names = []  # Список имен шарниров робота

    # Функция для старта симуляции в PyBullet
    def start_simulation(self):
        rospy.loginfo("Starting PyBullet Simulation")

        # Получаем путь к файлу URDF
        rospack = rospkg.RosPack()
        path = rospack.get_path('pybullet_ros_server')
        filename = os.path.join(path, 'urdf', 'testbot.xacro')
        urdf_filename = os.path.join(path, 'urdf', 'testbot.urdf')

        # Конвертируем XACRO в URDF файл
        urdf = open(urdf_filename, "w")
        subprocess.call(['rosrun', 'xacro', 'xacro', filename], stdout=urdf)

        # Подключаемся к PyBullet с графическим интерфейсом
        p.connect(p.GUI) #p.DIRECT) 
        p.setGravity(0, 0, self.GRAVITY)  # Устанавливаем значение силы тяжести
        # p.setPhysicsEngineParameter(enableConeFriction = 1)  # Включаем конусное трение (по умолчанию)

        # Загружаем пол в симуляцию
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Добавляем путь к данным PyBullet
        p.loadURDF("plane.urdf")  # Загружаем плоскость

        # Загружаем URDF модель робота
        self.robot_id = p.loadURDF(urdf_filename, basePosition=[0, 0, 0.1])

        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)  # Получаем информацию о шарнире
            joint_name = joint_info[1].decode("utf-8")  # Декодируем имя шарнира
            self.joint_names.append(joint_name)  # Добавляем имя шарнира в список

            # Добавляем отладочные линии для отображения осей шарниров в симуляции
            p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=self.robot_id, parentLinkIndex=i)  # Ось X
            p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=self.robot_id, parentLinkIndex=i)  # Ось Y
            p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=self.robot_id, parentLinkIndex=i)  # Ось Z
            
            # Если шарнир связан с колесом, изменяем его параметры трения
            if "wheel" in joint_name.lower():
                p.changeDynamics(self.robot_id, i, lateralFriction=0.5)

        # Главный цикл симуляции
        while not rospy.is_shutdown():
            self.publish_joint_velocities()  # Публикуем состояния шарниров
            p.stepSimulation()  # Шаг симуляции
            time.sleep(0.01)  # Пауза между шагами симуляции

    # Обработчик команды движения робота
    def move_robot(self, msg):
        rospy.loginfo("Received movement command: linear_x = %f, angular_z = %f", msg.linear.x, msg.angular.z)
        
        # Получаем линейную и угловую скорости
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Вычисляем скорости колес для движения робота
        left_wheels_speed = (linear_velocity - (angular_velocity * self.wheel_distance)/2) / self.wheel_radius
        right_wheels_speed = (linear_velocity + (angular_velocity * self.wheel_distance)/2) / self.wheel_radius
            
        l_f_motor_id = 1  # Идентификатор мотора левого переднего колеса
        l_r_motor_id = 2  # Идентификатор мотора левого заднего колеса
        r_f_motor_id = 3  # Идентификатор мотора правого переднего колеса
        r_r_motor_id = 4  # Идентификатор мотора правого заднего колеса

        # Устанавливаем целевые скорости для каждого мотора колес
        p.setJointMotorControl2(self.robot_id, l_f_motor_id, p.VELOCITY_CONTROL, targetVelocity=left_wheels_speed)
        p.setJointMotorControl2(self.robot_id, l_r_motor_id, p.VELOCITY_CONTROL, targetVelocity=left_wheels_speed)
        p.setJointMotorControl2(self.robot_id, r_f_motor_id, p.VELOCITY_CONTROL, targetVelocity=right_wheels_speed)
        p.setJointMotorControl2(self.robot_id, r_r_motor_id, p.VELOCITY_CONTROL, targetVelocity=right_wheels_speed)

    # Публикация состояния шарниров робота
    def publish_joint_velocities(self):
        joint_state = JointState()  # Создаем сообщение состояния шарниров
        velocities = []  # Список для хранения угловых скоростей шарниров
        names = []  # Список для хранения имен шарниров

        # Перебираем все шарниры и получаем их скорости
        for i in range(p.getNumJoints(self.robot_id)):
            joint_velocity = p.getJointState(self.robot_id, i)[1]  # Получаем скорость шарнира
            velocities.append(joint_velocity)  # Добавляем скорость в список
            names.append(self.joint_names[i])  # Добавляем имя шарнира в список

        joint_state.name = names  # Устанавливаем имена шарниров
        joint_state.velocity = velocities  # Устанавливаем скорости шарниров

        # Публикуем сообщение
        self.joint_state_publisher.publish(joint_state)
        rospy.loginfo("Published joint velocities: %s", velocities)  # Вывод опубликованных данных

    # Настройка ROS и запуск симуляции
    def setup_ros(self):
        rospy.init_node('pybullet_ros_server', anonymous=True)  # Инициализация ROS узела
        rospy.Subscriber('/cmd_vel', Twist, self.move_robot)  # Подписываемся на команду движения
        self.joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)  # Публикуем состояния шарниров
        self.start_simulation()  # Запускаем симуляцию

# Точка входа программы
if __name__ == "__main__":
    server = PyBulletRosServer()  # Создаем объект
    server.setup_ros()  # Настройка ROS и запуск симуляции
