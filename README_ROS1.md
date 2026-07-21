# GFS-X ROS1 Bridge

WIP-ветка для перехода GFSX_Robot на ROS 1 Noetic через `ROS-TCP-Connector`.

## Что входит

- Unity bridge для `Assets/GFSX/ROS1`
- Docker-конфигурация для ROS1 Noetic в `ros1-docker`
- Зафиксированная версия официального `ROS-TCP-Endpoint`
- Скрипты запуска, проверки и остановки

## Требования

- Docker Desktop
- Linux containers
- Unity-проект `Auto_M2M_Relay`
- ROS TCP endpoint на `127.0.0.1:10000`

## Запуск Docker

```powershell
cd C:\Users\Константин\Desktop\Auto_M2M_Relay\ros1-docker
.\start_ros1.ps1
```

Если политика выполнения блокирует скрипт:

```powershell
powershell -ExecutionPolicy Bypass -File .\start_ros1.ps1
```

Проверка:

```powershell
.\status_ros1.ps1
Test-NetConnection 127.0.0.1 -Port 10000
```

Ожидается контейнер `gfsx-ros1-endpoint` со статусом `healthy`.

## Unity-настройка

1. Открой проект и дождись компиляции новых файлов.
2. Выбери `Robotics > ROS Settings`.
3. Установи `Protocol = ROS1`.
4. После перекомпиляции выполни `Tools > URFU > Configure ROS1 Bridge`.
5. Затем выполни `Tools > URFU > Validate ROS1 Bridge`.
6. Сохрани сцену.

Важно:

- В ROS-сцене нужно отключить `Listen For TF Messages`.
- ROS-сцена и сцена ML-Agents должны быть отдельными.

## Топики

- `/gfsx/cmd_vel` - `geometry_msgs/Twist`
- `/gfsx/gripper/command` - `std_msgs/Bool`
- `/gfsx/ultrasonic/front` - `sensor_msgs/Range`
- `/gfsx/ir/left` - `std_msgs/Bool`
- `/gfsx/ir/right` - `std_msgs/Bool`
- `/gfsx/ir/gripper` - `std_msgs/Bool`
- `/gfsx/gripper/has_ball` - `std_msgs/Bool`

## Проверка движения

После запуска сцены в Unity в Console должен появиться адрес:

```text
GFS-X ROS1 bridge: 127.0.0.1:10000, cmd_vel=/gfsx/cmd_vel
```

Подача команды:

```powershell
cd C:\Users\Константин\Desktop\Auto_M2M_Relay\ros1-docker
docker compose exec ros1 bash -lc "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && timeout 3s rostopic pub -r 10 /gfsx/cmd_vel geometry_msgs/Twist \"linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}\""
```

Если bridge работает корректно, робот едет примерно три секунды и останавливается.

