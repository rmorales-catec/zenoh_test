#!/bin/zsh

# Configuración de RMW (solo Zenoh)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
LOG_DIR="$HOME/zenoh_test/Resultados"
NETWORK_CONFIGS=("dos_routers" "un_router" "peer_to_peer")

cd ~/ros2_ws
source install/setup.zsh

# Bucle para probar todas las configuraciones
for CONFIG in "${NETWORK_CONFIGS[@]}"; do
    echo "\n🌐 Probando configuración de red: $CONFIG"

    # Configurar la red según el caso
    case $CONFIG in
        "dos_routers")
            echo "🔧 Configurando para dos routers..."
            # Iniciar Zenoh
            echo "🚀 Iniciando Zenoh con configuración $CONFIG..."
            sudo fuser -k 7447/tcp
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_ROUTER_CONFIG_URI=$HOME/zenoh_test/Config/router_config_raspy.json5
            export ZENOH_ROUTER_CHECK_ATTEMPTS=
            export ZENOH_SESSION_CONFIG_URI=
            ros2 run rmw_zenoh_cpp rmw_zenohd &
            ZENOH_PID=$!
            sleep 2            
            ;;
        "un_router")
            echo "🔧 Configurando para un router..."
            # Iniciar Zenoh
            echo "🚀 Iniciando Zenoh con configuración $CONFIG..."
            sudo fuser -k 7447/tcp
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_ROUTER_CONFIG_URI=
            export ZENOH_ROUTER_CHECK_ATTEMPTS=
            export ZENOH_SESSION_CONFIG_URI=$HOME/zenoh_test/Config/session_config_router_raspy.json5
            sleep 2  
            ;;
        "peer_to_peer")
            echo "🔧 Configurando para conexión peer-to-peer..."
            sudo fuser -k 7447/tcp
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_ROUTER_CONFIG_URI=
            export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
            export ZENOH_SESSION_CONFIG_URI=$HOME/zenoh_test/Config/session_config_no_router_raspy.json5
            ;;
    esac

    # Limpiar estado anterior
    echo "🧹 Limpiando estado anterior..."
    ros2 daemon stop
    sleep 2

    echo "Nodo de LiDAR"
    cd ~/ros2_ws/src/ws_livox
    source install/setup.zsh
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py &
    LIVOX_PID=$!
    sleep 5  # tiempo para que inicien
    echo "Nodo de LiDAR ejecutandose"

    # Lanzar nodos en background
    echo "🚀 Lanzando nodos con $CONFIG..."
    echo "Nodo de imagenes"
    cd ~/ros2_ws
    source install/setup.zsh
    ros2 run prueba_rmw image_publisher_compressed &
    IMAGE_PID=$!
    sleep 2  # tiempo para que inicien
    echo "Nodo de imagenes ejecutandose"

    sleep 35

    # Finalizar nodos
    echo "🛑 Matando nodos..."

    # Cerrar el subscriber (usa cámara) completamente
    pkill -f image_publisher_compressed
    sleep 1
    while pgrep -f image_publisher_compressed > /dev/null; do
        echo "⏳ Esperando que image_publisher termine..."
        sleep 1
    done
    echo "✅ image_publisher cerrado"

    pkill -f livox_ros_driver2
    sleep 1
    while pgrep -f livox_ros_driver2 > /dev/null; do
        echo "⏳ Esperando que livox/lidar termine..."
        sleep 1
    done
    echo "✅ livox/lidar cerrado"

    sleep 2
    pkill -f rmw_zenohd
    sleep 1
    while pgrep -f rmw_zenohd > /dev/null; do
        echo "⏳ Esperando que router zenoh termine..."
        sleep 1
    done
    echo "✅ Nodo rmw_zenohd cerrado"

    sleep 2
    echo "Prueba con $CONFIG terminada"
    sleep 2
done

echo "✅ Pruebas completadas. Resultados en: $LOG_DIR"
