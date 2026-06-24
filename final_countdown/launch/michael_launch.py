from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():

    # --- Nodos que corren desde el inicio ---

    aruco_simu = Node(
        package='final_countdown',
        executable='aruco_simu',
        name='aruco_detector',
        output='screen'
    )

    baby_aruco = Node(
        package='final_countdown',
        executable='baby_aruco',
        name='aruco_detector',
        output='screen'
    )

    localisation = Node(
        package='final_countdown',
        executable='localisation',
        name='localisation',
        output='screen'
    )

    bug0 = Node(
        package='final_countdown',
        executable='bug0',
        name='reactive_navigation',
        output='screen'
    )

    # --- Primer nodo de alineacion (usa la camara, debe terminar antes del siguiente) ---

    alineation_ini = Node(
        package='final_countdown',
        executable='alineation_puzzle_ini',
        name='calibration_node',
        output='screen'
    )

    # --- Segundo nodo de alineacion (se lanza SOLO cuando el primero termina) ---

    alineation = Node(
        package='final_countdown',
        executable='alineation_puzzle',
        name='calibration_node',
        output='screen'
    )

    # Cuando alineation_ini termina (raise SystemExit), lanza alineation
    launch_alineation_on_ini_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=alineation_ini,
            on_exit=[alineation]
        )
    )

    return LaunchDescription([
        aruco_simu,
        baby_aruco,
        localisation,
        bug0,
        alineation_ini,
        launch_alineation_on_ini_exit,
    ])