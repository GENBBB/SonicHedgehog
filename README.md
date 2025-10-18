# SonicHedgehog (ROS2, Python)

## Цель
Разработать систему для мобильного робота, который **на скорость ищет центр неизвестного лабиринта**.  
Задача решается в ROS2, с поддержкой:
- 2D SLAM для построения карты лабиринта,
- глобального планирования маршрута через D*,
- локального планирования с помощью нейросети,
- ручного управления и визуализации (RViz).


## 1. Выбранный SLAM
- **SLAM Toolbox (ROS2)** — основной SLAM для задачи (2D карта лабиринта).  
- Дополнительно можно подключать **RTAB-Map** или **Cartographer**, но они не основные.  

Выход: `nav_msgs/OccupancyGrid` (карта), `/odom`, `/map -> /odom` трансформации.


## 2. Архитектура системы

### Основные узлы:
- **slam_toolbox_interface** — SLAM (2D).
- **global_planner_dstar** — построение маршрута.
- **keypoint_manager** — выделение ключевых точек.
- **local_planner_nn** — локальное планирование с нейросетью.
- **visualization** — RViz2 конфигурации.

## 3. Зависимости

### ROS2-пакеты
- `slam_toolbox` (2D SLAM)  
- `nav2_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`  
- `ros2_control`, `robot_state_publisher`  
- `rviz2`  

### Python-библиотеки
- `rclpy` — ROS2 для Python  
- `numpy`, `scipy` — вычисления  
- `networkx` — реализация D* (графы, пути)  
- `torch` или `tensorflow` — нейросеть для локального планировщика  
- `opencv-python` — обработка изображений (опционально для RealSense)  

### Инструменты 
- Intel RealSense SDK (если используется RGB-D)  


## 4. Структура проекта

```

SonicHedgehog/                        # Основной репозиторий
├── README.md                               # описание проекта
├── launch/                                 # launch-файлы
│   ├── sim_bringup.launch.py               # запуск симулятора
│   ├── real_bringup.launch.py              # запуск реального робота
│   └── teleop_test.launch.py               # ручное управление
├── configs/                                # параметры (YAML)
│   ├── slam_toolbox.yaml                   # настройки SLAM
│   ├── dstar.yaml                          # настройки глобального планировщика
│   ├── local_planner_nn.yaml               # конфигурация нейросети
│   └── robot_params.yaml                   # параметры робота (размеры, динамика)
├── src/                               # ROS2-пакеты (colcon workspace)
│   ├── slam_toolbox_interface/             # адаптер для SLAM Toolbox
│   │   └── slam_wrapper_node.py
│   ├── global_planner_dstar/               # глобальное планирование (D*)
│   │   ├── dstar_node.py
│   │   └── utils/graph_utils.py
│   ├── keypoint_manager/                   # управление ключевыми точками
│   │   └── keypoint_node.py
│   ├── local_planner_nn/                   # локальный планировщик (NN)
│   │   ├── nn_node.py
│   │   ├── model_loader.py
│   │   └── preprocessor.py
│   ├── webots_adapter/                     # новый пакет для интеграции
│   │   ├── adapter_node.py                 # подписывается на webots-топики
│   │   ├── sim_telemetry_node.py           # публикует /odom, /scan
│   │   └── sim_cmdvel_node.py              # принимает /cmd_vel
│   ├── supervisor/                         # безопасность и арбитраж
│   │   └── supervisor_node.py
│   └── visualization/                      # RViz2
│       ├── rviz_config.rviz
│       └── visualization_node.py           # опциональные маркеры в RViz
└── models/                                 # модели нейросети
    └── nn_local_planner/
        ├── latest_model.pth
        └── metadata.yaml

````

## 5. Сценарии запуска
* **Симуляция**  
  ```bash
  ros2 launch SonicHedgehog sim_bringup.launch.py
  ```

* **Ручное управление (тесты)**

  ```bash
  ros2 launch SonicHedgehog teleop_test.launch.py
  ```


## 6. Принципы расширяемости

* Каждый пакет реализует **ROS2-интерфейс с фиксированным API** (`/odom`, `/map`, `/cmd_vel`, `/scan`).
* Можно заменять SLAM, планировщики, контроллер или драйвер без изменения других модулей.
* Поддержка **симуляции и реального робота через единый интерфейс**.
* Использование `ros2 lifecycle` для управляемых нод (инициализация/пауза/рестарт).
* Параметры вынесены в YAML-конфиги.
