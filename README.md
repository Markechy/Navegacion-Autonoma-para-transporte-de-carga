# Navegación Autónoma para Transporte de Carga con Puzzlebot

Sistema de navegación autónoma desarrollado para el **Puzzlebot LiDAR Edition** (Manchester
Robotics), capaz de detectar, recoger, transportar y depositar un cubo de carga de forma
completamente autónoma, evadiendo obstáculos en el trayecto y regresando a su posición inicial.

## Descripción del reto

El robot inicia en una posición intermedia entre dos estaciones (carga y descarga) separadas ~2 m
y delimitadas con marcas de 30×30 cm. El cubo lleva un marcador ArUco para su identificación.
La misión consiste en:

1. Detectar el ArUco del cubo y estimar su pose relativa.
2. Alinearse y recoger el cubo con un mecanismo de pala.
3. Navegar hacia la estación de descarga, evadiendo obstáculos imprevistos.
4. Depositar el cubo lo más cerca posible del centro del área.
5. Regresar de forma autónoma al punto de origen.

## Arquitectura

El sistema se compone de tres subsistemas principales comunicados vía tópicos de ROS2:

- **Localización** — odometría diferencial corregida con un **Filtro de Kalman Extendido (EKF)**,
  usando los ArUco de posición global conocida como referencia para reducir la deriva.
- **Detección visual** — detección de marcadores **ArUco** con OpenCV. Cámara calibrada con
  checkerboard; estimación de pose con `solvePnP`. Diccionarios distintos para entorno (4×4) y
  cubo de carga (5×5).
- **Navegación reactiva** — algoritmo **Bug0**: avance hacia la meta con controlador proporcional
  y evasión de obstáculos por LiDAR.

## Nodos principales

| Nodo | Función |
|------|---------|
| `alineation_puzzle_init` | Alineación inicial al ArUco de carga (DICT_4X4, ID 0). FSM: searching → aligning → waiting. Control proporcional (kw=0.003). |
| `alineation_puzzle` | Alineación y empuje del cubo (DICT_5X5, ID 34). Extiende la FSM con advancing y pushing. |
| `aruco_simu` | Detección de ArUcos de referencia (DICT_4X4_100, 0.1 m). Publica pose en el marco global → `/aruco_poses`. |
| `baby_aruco` | Detección del ArUco del cubo (DICT_5X5_100, 0.03 m). Publica pose en el marco del robot. |
| `bug0` | Núcleo de navegación. FSM de 7 estados: send_heading, stop_robot, go_to_goal, follow_wall, turning, collecting_minibox, drop_box. |
| `localisation` | Estimación de pose con odometría diferencial + EKF (predicción y corrección). |

## Detalles técnicos

- **Modelo cinemático:** tracción diferencial. R = 0.0473 m (radio de rueda), L = 0.182 m (distancia entre ruedas).
- **EKF:** matrices de covarianza `L_k` (proceso/encoders) y `R_k` (medición/ArUco) obtenidas
  experimentalmente por linealización sobre datos del hardware real.
- **Navegación:** controlador proporcional sobre error de posición y orientación.
- **Evasión Bug0:** LiDAR dividido en 6 sectores; seguimiento de contorno a 0.25 m, transición a
  evasión con obstáculo frontal a < 0.3 m.

## Resultados

**Simulación (Gazebo) — 8 waypoints:**
- Error euclidiano promedio: **0.054 m** (máx. 0.072 m, mín. 0.042 m).
- El controlador converge consistentemente a < 8 cm del objetivo.

**Implementación física — 17 ejecuciones:**
- 14 exitosas / 3 fallidas → **tasa de éxito 82.35 %**.

## Ventajas y limitaciones

**Ventajas:** arquitectura modular (nodos independientes, fáciles de depurar y escalar); ArUco + EKF
compensan la deriva de la odometría; Bug0 reacciona a obstáculos sin necesidad de mapa previo.

**Limitaciones:** alta dependencia de los ArUco (ángulos de visión desfavorables degradan la
localización); Bug0 puede quedar atrapado en obstáculos cóncavos o laberintos; la alineación por
visión es sensible a variaciones en la posición inicial del cubo.

## Tecnologías

- **Framework:** ROS2
- **Visión:** OpenCV (detección ArUco, `solvePnP`, calibración con checkerboard)
- **Simulación:** Gazebo
- **Hardware:** Puzzlebot LiDAR Edition (Manchester Robotics) — cámara, LiDAR, encoders, servo de pala

## Autores

* Ivan A. Melo Salgado  
* Marco A. González Fernández  
* Marisol S. Ramírez Herrera  

## Referencias

- Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
- Manchester Robotics. TE3003B: Integration of Robotics and Intelligent Systems.
