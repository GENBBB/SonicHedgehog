import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, DriveOnHeading
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from tf_transformations import quaternion_from_euler
import threading
import time
import math

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node_nav2')
        
        # Action клиенты для всех типов движения через Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        
        # Подписка на текущую позицию для вычисления относительных движений
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            10
        )

        self.mission = [
            ('navigate_to_pose', (0.0, 0.0, math.pi)),
            ('navigate_to_pose', (-2.0, 0.0, math.pi)),
            ('navigate_to_pose', (-5.2, 4.8, math.radians(-287))),
            ('navigate_to_pose', (-4.3, 6.7, math.radians(-321))),
            ('navigate_to_pose', (-0.5, 8.4, math.radians(-270))),
            ('navigate_to_pose', (-0.5, 12.3, math.radians(-270))),
            ('navigate_to_pose', (-0.5, 15.5, math.radians(-270))),
            ('navigate_to_pose', (-3.6, 16.1, math.radians(-217))),
            ('navigate_to_pose', (-4.2, 18.2, math.radians(-274))),
            ('navigate_to_pose', (-0.0, 20.0, math.radians(-355))),

        ]

        # Запуск миссии после инициализации
        self.timer = self.create_timer(15.0, self.start_mission)

    def pose_callback(self, msg):
        """Сохраняем текущую позицию робота"""
        self.current_pose = msg.pose.pose

    def start_mission(self):
        """Запуск выполнения миссии в отдельном потоке"""
        self.destroy_timer(self.timer)
        self.get_logger().info('Начало выполнения миссии')
        
        # Запускаем миссию в отдельном потоке, чтобы не блокировать основной
        mission_thread = threading.Thread(target=self.execute_mission)
        mission_thread.daemon = True
        mission_thread.start()

    def execute_mission(self):
        """Выполнение миссии в отдельном потоке"""
        for i, (action, value) in enumerate(self.mission):
            self.get_logger().info(f'Шаг {i+1}/{len(self.mission)}: {action}')
            
            success = False
            try:
                if action == 'rotate_to':
                    success = self.execute_rotate_to(value)
                elif action == 'drive':
                    success = self.execute_drive(value)
                elif action == 'navigate_to_pose':
                    x, y, theta = value
                    success = self.execute_navigate_to_pose(x, y, theta)
                else:
                    self.get_logger().error(f'Неизвестный тип действия: {action}')
                    continue
                
                if success:
                    self.get_logger().info(f'Шаг {i+1} завершен успешно')
                else:
                    self.get_logger().error(f'Шаг {i+1} завершен с ошибкой')
                    break
                    
                # Короткая пауза между действиями
                time.sleep(1.0)
                
            except Exception as e:
                self.get_logger().error(f'Ошибка на шаге {i+1}: {str(e)}')
                break

        self.get_logger().info('Миссия завершена')

    def execute_rotate_to(self, target_yaw):
        """Поворот к заданной ориентации (абсолютный угол) через NavigateToPose"""
        self.get_logger().info(f'Поворот к ориентации {target_yaw:.2f} рад')
        
        if self.current_pose is None:
            self.get_logger().error('Текущая позиция не известна')
            return False
            
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Сервер NavigateToPose не доступен')
            return False

        # Создаем цель с текущей позицией и целевой ориентацией
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(
            self.current_pose.position.x,
            self.current_pose.position.y,
            target_yaw
        )
        goal_msg.behavior_tree = ''  # использовать дефолтное поведение
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        return self.wait_for_action_result(future, 'RotateTo')

    def execute_drive(self, distance):
        """Выполнение движения вперед через Nav2 DriveOnHeading action"""
        self.get_logger().info(f'Движение вперед на {distance:.2f} м')
        
        if not self.drive_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Сервер Drive не доступен')
            return False

        goal_msg = DriveOnHeading.Goal()
        goal_msg.target.x = distance
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = 0.2  # м/с
        goal_msg.time_allowance = Duration(seconds=10.0)
        
        future = self.drive_client.send_goal_async(goal_msg)
        return self.wait_for_action_result(future, 'Drive')

    def execute_navigate_to_pose(self, x, y, theta):
        """Навигация к точке с заданной позицией и ориентацией"""
        self.get_logger().info(f'Навигация к точке ({x:.2f}, {y:.2f}) с ориентацией {theta:.2f}')
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Сервер NavigateToPose не доступен')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, theta)
        goal_msg.behavior_tree = ''  # использовать дефолтное поведение
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        return self.wait_for_action_result(future, 'NavigateToPose')

    def create_pose_stamped(self, x, y, theta):
        """Создание PoseStamped сообщения"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Преобразуем эйлеров угол в кватернион
        quat = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose

    def wait_for_action_result(self, future, action_name):
        """Ожидание результата от action сервера (синхронная версия)"""
        # Ждем завершения future
        start_time = time.time()
        while not future.done():
            time.sleep(0.1)
            if time.time() - start_time > 30:  # таймаут 30 секунд
                self.get_logger().error(f'Таймаут ожидания {action_name}')
                return False
        
        try:
            # Получаем результат
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'{action_name} цель отклонена')
                return False
            
            self.get_logger().info(f'{action_name} цель принята')
            
            # Ждем завершения выполнения
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            while not result_future.done():
                time.sleep(0.1)
                if time.time() - start_time > 60:  # таймаут 60 секунд
                    self.get_logger().error(f'Таймаут выполнения {action_name}')
                    return False
            
            result = result_future.result()
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'{action_name} завершено успешно')
                return True
            else:
                status_name = self.get_goal_status_name(result.status)
                self.get_logger().warn(f'{action_name} завершено с статусом: {status_name} ({result.status})')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Ошибка при выполнении {action_name}: {str(e)}')
            return False

    def get_goal_status_name(self, status):
        """Получить текстовое название статуса цели"""
        status_names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED'
        }
        return status_names.get(status, f'UNKNOWN_STATUS_{status}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MissionNode()
        
        # Используем стандартный исполнитель
        rclpy.spin(node)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()