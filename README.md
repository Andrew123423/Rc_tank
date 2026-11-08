# RC Tank — ROS 2 Simulation

Симуляция гусеничной платформы с двухступенчатой коробкой передач для **ROS 2 Jazzy**, **Ubuntu 24.04** и **Gazebo Harmonic**. Модель создана по STL проекта [HowToMechatronics](https://howtomechatronics.com/projects/making-a-beast-of-a-3d-printed-rc-tank-with-dual-speed-gearbox/); геометрия упрощена для снижения нагрузки.



## Сборка

Поместите пакет в `~/tank_ws/src/rc_tank_sim`. После установки ROS 2 Jazzy выполните:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/tank_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select rc_tank_sim
source install/setup.bash
```

## Запуск

Gazebo и RViz с включёнными камерами и лидаром:

```bash
ros2 launch rc_tank_sim sim.launch.py \
  profile:=light gui:=true rviz:=true \
  sensors:=true lidar:=true cameras:=true
```

В отдельном терминале с подключённым окружением ROS и рабочего пространства:

```bash
ros2 run rc_tank_sim keyboard_teleop
```

Кликните в окно управления: **W/S** — движение, **A/D** — поворот, **W+A / W+D** — движение по дуге, **1/2** — передача, **F** — свет, **пробел/Esc** — остановка. В RViz включите панели `Left stereo - optional` и `Right stereo - optional` для просмотра изображений.

