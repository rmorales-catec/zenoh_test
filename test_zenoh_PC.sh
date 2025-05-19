#!/bin/zsh

INTERFACE="enxc84d44228958"  # Cambia esto por tu interfaz de red

# Configuración de RMW (solo Zenoh)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
DURATION=25  # segundos que se escucha
LOG_DIR="$HOME/zenoh_test/Resultados"
NETWORK_CONFIGS=("dos_routers" "un_router" "peer_to_peer")

mkdir -p "$LOG_DIR"
cd ~/ros2_ws
source install/setup.zsh

# Función que espera a que existan ambos topics
wait_for_topics() {
    echo "🔎 Esperando a que los topics estén disponibles..."

    for i in {1..60}; do
        # HAS_IMAGE=$(ros2 topic list 2>/dev/null | grep -q "/image_compressed" && echo "1" || echo "0")
        HAS_LIVOX=$(ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && echo "1" || echo "0")

        if [[ "$HAS_LIVOX" == "1" ]]; then  # "$HAS_IMAGE" == "1" &&
            echo "✅ Ambos topics disponibles"
            sleep 5
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ No se detectaron ambos topics tras 60 segundos"
    return 1
}

wait_for_topics2() {
    echo "🔎 Esperando a que los topics estén disponibles..."

    for i in {1..60}; do
        HAS_IMAGE=$(ros2 topic list 2>/dev/null | grep -q "/image_compressed" && echo "1" || echo "0")
        HAS_LIVOX=$(ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && echo "1" || echo "0")
        HAS_COMPRESSED=$(ros2 topic list 2>/dev/null | grep -q "/livox/lidar/compressed" && echo "1" || echo "0")
        HAS_DRACO=$(ros2 topic list 2>/dev/null | grep -q "/pointcloud_draco" && echo "1" || echo "0")


        if [[ "$HAS_IMAGE" == "1" && "$HAS_LIVOX" == "1" && "$HAS_COMPRESSED" == "1" && "$HAS_DRACO" == "1"  ]]; then
            echo "✅ Todos los topics disponibles"
            sleep 5
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ No se detectaron los topics tras 60 segundos"
    return 1
}

mover_pcaps() {
    echo "📦 Moviendo archivos .pcap desde /tmp a $LOG_DIR"

    local files=(/tmp/*.pcap)

    if [[ ${#files[@]} -eq 0 ]]; then
        echo "⚠️ No se encontraron archivos .pcap en /tmp"
        return
    fi

    for file in "${files[@]}"; do
        echo "➡️  Moviendo $(basename "$file")"
        sudo mv "$file" "$LOG_DIR/"
    done

    # Aseguramos que el usuario tenga permisos sobre los archivos
    sudo chown $USER:$USER "$LOG_DIR"/*.pcap

    echo "✅ Todos los archivos .pcap fueron movido"
}

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
            export ZENOH_ROUTER_CONFIG_URI=$HOME/zenoh_test/Config/router_config_PC.json5
            export ZENOH_ROUTER_CHECK_ATTEMPTS=0
            sleep 1
            cd ~/ros2_ws
            source install/setup.zsh
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
            export ZENOH_ROUTER_CHECK_ATTEMPTS=0
            # export ZENOH_ROUTER_CONFIG_URI=$HOME/zenoh_test/Config/router_config_PC.json5
            sleep 1
            cd ~/ros2_ws
            source install/setup.zsh
            ros2 run rmw_zenoh_cpp rmw_zenohd &
            ZENOH_PID=$!
            sleep 2  
            ;;
        "peer_to_peer")
            echo "🔧 Configurando para conexión peer-to-peer..."
            sudo fuser -k 7447/tcp
            sleep 1
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_SESSION_CONFIG_URI=$HOME/zenoh_test/Config/session_config_no_router_PC.json5
            export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
            ;;
    esac

    # Parar el daemon para evitar caché anterior
    ros2 daemon stop
    sleep 2

    # Esperar a que ambos topics estén disponibles
    if ! wait_for_topics; then
        echo "⚠️ Saltando $CONFIG por timeout en topics"
        continue
    fi

    # sleep 4
    echo "📏 Midiendo frecuencia y delay de la imagen..."
    timeout ${DURATION}s ros2 topic hz /image_compressed -w 30 | tee "$LOG_DIR/${CONFIG}_image_hz.txt" &
    HZ_IMAGE_PID=$!
    echo "📏 Midiendo frecuencia y delay del lidar..."
    timeout ${DURATION}s ros2 topic hz /livox/lidar -w 30 | tee "$LOG_DIR/${CONFIG}_lidar_hz.txt" &
    HZ_LIDAR_PID=$!

    timeout ${DURATION}s ros2 topic delay /image_compressed -w 30 | tee "$LOG_DIR/${CONFIG}_image_delay.txt" &
    DELAY_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic delay /livox/lidar -w 30 | tee "$LOG_DIR/${CONFIG}_lidar_delay.txt" &
    DELAY_LIDAR_PID=$!

    timeout ${DURATION}s ros2 topic bw /image_compressed | tee "$LOG_DIR/${CONFIG}_image_bw.txt" &
    BW_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic bw /livox/lidar | tee "$LOG_DIR/${CONFIG}_lidar_bw.txt" &
    BW_LIDAR_PID=$!

    # Lanzamos subscriber de imágenes
    echo "🚀 Lanzando nodo de imágenes..."
    cd ~/ros2_ws
    source install/setup.zsh
    sleep 1
    ros2 run prueba_rmw image_subscriber_compressed &
    SUB_PID=$!
    sleep 2

    echo "Lanzamos RViz con la configuración de Lidar..."
    RVIZ_CONFIG_PATH="$HOME/zenoh_test/Config/livox-lidar.rviz"
    # Verifica que el archivo de configuración exista
    if [[ -f "$RVIZ_CONFIG_PATH" ]]; then
        echo "🚀 Lanzando RViz con configuración predefinida..."
        rviz2 -d "$RVIZ_CONFIG_PATH" &
        RVIZ_PID=$!
        sleep 1
    else
        echo "⚠️ No se encontró el archivo de configuración de RViz en: $RVIZ_CONFIG_PATH"
    fi
    sleep 2

    (
        sleep 7
        echo "📡 Iniciando captura de paquetes con tcpdump..."
        TCPDUMP_TMP="/tmp/${CONFIG}.pcap"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"

        sudo tcpdump -i "$INTERFACE" -w "$TCPDUMP_TMP" &
        TCPDUMP_PID=$!
        echo "$TCPDUMP_PID" > "$TCPDUMP_PID_FILE"
        echo "Captura guardada en: $TCPDUMP_TMP (PID: $TCPDUMP_PID)"
        sleep 2
    ) &

    (
        sleep 13
        echo "🛑 Deteniendo captura de paquetes (auto)"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"
        if [ -f "$TCPDUMP_PID_FILE" ]; then
            TCPDUMP_PID=$(cat "$TCPDUMP_PID_FILE")
            if ps -p "$TCPDUMP_PID" > /dev/null; then
                sudo kill -SIGINT "$TCPDUMP_PID"
                echo "✅ Captura de paquetes cerrada (PID: $TCPDUMP_PID)"
            else
                echo "⚠️ tcpdump ya no está corriendo"
            fi
            rm "$TCPDUMP_PID_FILE"
            echo "PID eliminado"
        else
            echo "❌ Archivo PID no encontrado"
        fi
    ) &



    # Esperar a que se complete el tiempo
    wait $HZ_IMAGE_PID
    wait $HZ_LIDAR_PID
    wait $DELAY_IMAGE_PID
    wait $DELAY_LIDAR_PID
    wait $BW_IMAGE_PID
    wait $BW_LIDAR_PID

    echo "🛑 Deteniendo nodos..."
    # Detenemos el subscriber de imágenes
    pkill -f image_subscriber_compressed
    sleep 1

    # Espera hasta que el proceso se cierre completamente
    while pgrep -f image_subscriber_compressed > /dev/null; do
        echo "⏳ Esperando que image_subscriber termine..."
        sleep 1
    done
    echo "✅ Nodo image_subscriber cerrado"


    kill $RVIZ_PID
    echo "✅ RViz cerrado"

    sleep 2
    pkill -f rmw_zenohd
    sleep 1
    while pgrep -f rmw_zenohd > /dev/null; do
        echo "⏳ Esperando que router zenoh termine..."
        sleep 1
    done
    echo "✅ Nodo rmw_zenohd cerrado"


    echo "✅ Prueba completada para configuración $CONFIG"
    sleep 5
done

echo "📁 Pruebas completas. Resultados guardados en: $LOG_DIR"

mover_pcaps

# Generar gráficos
echo "📊 Generando gráficos..."
cd $HOME/zenoh_test
python3 graficos_zenoh.py
