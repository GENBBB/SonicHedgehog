import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, DriveOnHeading, NavigateThroughPoses
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from tf_transformations import quaternion_from_euler
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Empty
from slam_toolbox.srv import Reset
import threading
import time
import math

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node_nav2')
        
        # Action клиенты для всех типов движения через Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # Подписка на текущую позицию для вычисления относительных движений
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            10
        )

        self.cmdvel = self.create_publisher(Twist, 'cmd_vel', 10)

# ('navigate_through_poses', [
#             (0.0, 0.0, math.pi)),
#             (-2.0, 0.0, math.radians(145)),
#             (-3.0, 2.0, math.radians(145)),
#             (-5.2, 4.8, math.radians(-287)),
#             (-4.3, 6.7, math.radians(-321)),
#             (-2.8, 8.2, math.radians(0)),  
#             (-1.5, 8.4, math.radians(0)),
#             (-1.5, 8.4, math.radians(-270)),
# ]),


        self.mission = [
            # ('navigate_to_pose', (0.0, 0.0, math.pi)),
            # ('navigate_to_pose', (-2.0, 0.0, math.radians(145))),
            # ('navigate_to_pose', (-3.0, 2.0, math.radians(145))),
            # ('navigate_to_pose', (-5.2, 4.8, math.radians(-287))),
            # ('navigate_to_pose', (-4.3, 6.7, math.radians(-321))),
            # ('navigate_to_pose', (-2.8, 8.2, math.radians(0))),  
            # ('navigate_to_pose', (-1.5, 8.4, math.radians(0))),
            # ('navigate_to_pose', (-1.5, 8.4, math.radians(-270))),
            ('navigate_through_poses', [
                        # (0.0, 0.0, math.pi),
                        (-2.5, 0.0, math.radians(145)),
                        (-3.0, 2.0, math.radians(145)),
                        (-5.2, 4.8, math.radians(-287)),
                        (-4.3, 6.7, math.radians(-321)),
                        # (-2.8, 8.2, math.radians(0)),  
                        # (-1.5, 8.4, math.radians(0)),
                        (-1.5, 8.1, math.radians(-90)),
                        (-1.5, 11.1, math.radians(-90)),
                        (-1.5, 15.3, math.radians(-90)),
            ]),

            ('clear_costmap', (None,)),
            ('clear_slam_map', (None,)),

            ('navigate_through_poses', [
                        (-3.5, 15.0, math.radians(-180)),
                        (-4.7, 15.0, math.radians(-180)),
                        (-4.7, 18.0, math.radians(-180)),
                        (-0.0, 20.0, math.radians(-355)),
            ]),

            ('navigate_through_poses', [
                        # (0.0, 0.0, math.pi),
                        #(-2.0, 0.0, math.radians(145)),
                        #(-3.0, 2.0, math.radians(145)),
                        #(-5.2, 4.8, math.radians(-287)),
                        #(-4.3, 6.7, math.radians(-321)),
                        # (-2.8, 8.2, math.radians(0)),  
                        # (-1.5, 8.4, math.radians(0)),
                        #(-1.5, 8.1, math.radians(0)),
                        #(-2.0, 15.3, math.radians(-217)),
                        #(-3.5, 15.3, math.radians(-217)),
                        (-5.7, 15.3, math.radians(-274)),
                        (-5.7, 18.0, math.radians(-274)),
                        (-0.0, 18.0, math.radians(-355)),
            ]),

            # ('navigate_through_poses', [
            #             # (0.0, 0.0, math.pi),
            #             (-2.0, 0.0, math.radians(145)),
            #             (-4.767, 5.185, math.radians(145)),
            #             (-4.809, 6.411, math.radians(-287)),
            #             (-2.45, 8.242, math.radians(-321)),
            #             # (-2.8, 8.2, math.radians(0)),  
            #             (-0.556, 8.142, math.radians(0)),
            #             (2.557, 9.631, math.radians(-270)),
            #             (6.358, 10.082, math.radians(-217)),
            #             (5.486, 14.02, math.radians(-274)),
            #             (9.215, 14.818, math.radians(-355)),
            #             (10.052, 12.02, math.radians(-355)),
            # ]),

            ('cmd_vel', (2.5, 20)),
            ('cmd_vel', (0.0, 20)),
            ('clear_costmap', (None,)),
            ('clear_slam_map', (None,)),
            ('navigate_to_pose', (-3.6, 16.3, math.radians(-217))),
            ('navigate_to_pose', (-4.2, 18.2, math.radians(-274))),
            ('navigate_to_pose', (-0.0, 20.0, math.radians(-355))),

        ]

        # Запуск миссии после инициализации
        self.timer = self.create_timer(10.0, self.start_mission)

    # 3) Фидбэк:
    def _ntp_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        #self.get_logger().info(
        #    f"[NTP] осталось поз: {fb.number_of_poses_remaining}, "
        #    f"дистанция: {fb.distance_remaining:.2f} м"
        #)

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
                elif action == 'navigate_through_poses':
                    waypoints = value  # список [(x,y,theta), ...]
                    success = self.execute_navigate_through_poses(waypoints)
                elif action == 'cmd_vel':
                    v, t = value
                    self.send_cmd_vel(v, t)
                    success = True
                elif action == 'clear_costmap':
                    self.clear_all_costmaps()
                    success = True
                elif action == 'clear_slam_map':
                    self.clear_slam_map()
                    success = True
                else:
                    self.get_logger().error(f'Неизвестный тип действия: {action}')
                    continue
                
                if success:
                    self.get_logger().info(f'Шаг {i+1} завершен успешно')
                else:
                    self.get_logger().error(f'Шаг {i+1} завершен с ошибкой')
                    break
                    
                # Короткая пауза между действиями
                time.sleep(0.2)
                
            except Exception as e:
                self.get_logger().error(f'Ошибка на шаге {i+1}: {str(e)}')
                break

        self.get_logger().info('Миссия завершена')
    
    def send_cmd_vel(self, linear_x=0.0, t=0):
        self.get_logger().info(f'Движение вперед {linear_x:.2f} м/с')
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        
        # Публикуем сообщение
        self.cmdvel.publish(msg)
        time.sleep(t)

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
    
    def clear_slam_map(self, timeout_sec: float = 5.0):
        srv_name = '/slam_toolbox/reset'
        self.get_logger().info(f'Сброс SLAM через сервис {srv_name} (таймаут {timeout_sec}s)...')

        client = self.create_client(Reset, srv_name)

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'Сервис {srv_name} недоступен (таймаут {timeout_sec}s).')
            return False

        # Формируем запрос (по дефолту pause_new_measurements=False)
        req = Reset.Request()
        req.pause_new_measurements = False

        try:
            future = client.call_async(req)
        except Exception as e:
            self.get_logger().error(f'Ошибка при отправке запроса к {srv_name}: {e}')
            return False

        # Синхронное ожидание ответа через spin_once()
        start_time = time.time()
        while not future.done():
            if (time.time() - start_time) >= timeout_sec:
                self.get_logger().error(f'Таймаут ожидания ответа от {srv_name} после {timeout_sec}s.')
                return False

        if future.done():
            try:
                _resp = future.result()
                self.get_logger().info(f'SLAM успешно сброшен ({srv_name}).')
                return True
            except Exception as e:
                self.get_logger().error(f'Ошибка в ответе от {srv_name}: {e}')
                return False
        else:
            self.get_logger().error(f'Таймаут ожидания ответа от {srv_name} после {timeout_sec}s.')
            return False


    def clear_all_costmaps(self, timeout_sec: float = 5.0):
        """
        Синхронно очищает глобальную и локальную costmap через nav2_msgs/ClearEntireCostmap.
        Блокирует выполнение и ждёт ответа от каждого сервиса (последовательно).
        timeout_sec — таймаут ожидания ответа для каждого сервиса в секундах.
        Возвращает True если оба сервиса ответили успешно.
        """
        self.get_logger().info('Синхронная очистка: глобальной и локальной costmap...')

        services = [
            ('/global_costmap/clear_entirely_global_costmap', 'глобальная'),
            ('/local_costmap/clear_entirely_local_costmap', 'локальная'),
        ]

        all_ok = True

        for srv_name, desc in services:
            client = self.create_client(ClearEntireCostmap, srv_name)

            # Ждём появления сервиса
            self.get_logger().info(f'Ожидание сервиса {srv_name} ({desc}) (таймаут {timeout_sec}s)...')
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f'Сервис {srv_name} ({desc}) недоступен (таймаут).')
                all_ok = False
                continue

            # Подготовка запроса (внутри ClearEntireCostmap.request — std_msgs/Empty)
            req = ClearEntireCostmap.Request()
            req.request = Empty()

            try:
                future = client.call_async(req)
            except Exception as e:
                self.get_logger().error(f'Ошибка при отправке запроса к {srv_name} ({desc}): {e}')
                all_ok = False
                continue

            # Блокируем ожидание ответа безопасным циклом (rclpy.spin_once)
            start_time = time.time()
            while not future.done():
                if (time.time() - start_time) >= timeout_sec:
                    break

            if future.done():
                try:
                    _resp = future.result()  # ClearEntireCostmap_Response (содержит std_msgs/Empty)
                    self.get_logger().info(f'{desc.capitalize()} costmap успешно очищена ({srv_name}).')
                except Exception as e:
                    self.get_logger().error(f'Ошибка в ответе от {srv_name} ({desc}): {e}')
                    all_ok = False
            else:
                self.get_logger().error(f'Таймаут ожидания ответа от {srv_name} ({desc}) после {timeout_sec}s.')
                all_ok = False

        if not all_ok:
            self.get_logger().warn('Очистка завершилась с ошибками (см. логи).')
        else:
            self.get_logger().info('Обе costmap успешно очищены.')
        time.sleep(1)
        return all_ok



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

    def execute_navigate_through_poses(self, waypoints, behavior_tree: str = ''):
        """
        waypoints: iterable[(x, y, theta)] в кадре 'map'
        behavior_tree: имя/путь BT-дерева или '' для дефолта
        """
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Сервер NavigateThroughPoses недоступен')
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.create_pose_stamped(x, y, th) for (x, y, th) in waypoints]
        goal.behavior_tree = behavior_tree  # можно оставить '' для дефолта

        future = self.nav_through_poses_client.send_goal_async(
            goal, feedback_callback=self._ntp_feedback_cb
        )
        return self.wait_for_action_result(future, 'NavigateThroughPoses')


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
                if time.time() - start_time > 500:  # таймаут 60 секунд
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