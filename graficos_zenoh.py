import matplotlib.pyplot as plt
import os
import re

log_dir = os.path.expanduser('~/zenoh_test/Resultados')

# Solo una RMW pero distintos escenarios
escenarios = ['peer_to_peer', 'un_router', 'dos_routers']

# --- Función para parsear archivos ---
def parse_file(filepath, mode='hz'):
    values = []
    if not os.path.exists(filepath):
        print(f"⚠️  No existe el archivo {filepath}")
        return values
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if mode == 'hz':
                m = re.search(r'average rate:\s+([0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
            elif mode == 'delay':
                m = re.search(r'average delay:\s+(-?[0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
            elif mode == 'bw':
                m = re.search(r'([0-9.]+)\s*(KB|MB)/s', line, re.IGNORECASE)
                if m:
                    value = float(m.group(1))
                    unit = m.group(2).upper()
                    if unit == 'KB':
                        value /= 1024.0
                    values.append(value)
                elif 'no new messages' in line:
                    values.append(0.0)
    print(f"{os.path.basename(filepath)} → {len(values)} muestras")
    return values

# --- Cargar datos ---
def cargar_datos(topic):
    datos_hz = {}
    datos_delay = {}
    datos_bw = {}
    for escenario in escenarios:
        datos_hz[escenario] = parse_file(os.path.join(log_dir, f"{escenario}_{topic}_hz.txt"), mode='hz')
        datos_delay[escenario] = parse_file(os.path.join(log_dir, f"{escenario}_{topic}_delay.txt"), mode='delay')
        datos_bw[escenario] = parse_file(os.path.join(log_dir, f"{escenario}_{topic}_bw.txt"), mode='bw')
    return datos_hz, datos_delay, datos_bw

data_image_hz, data_image_delay, data_image_bw = cargar_datos("image")
data_lidar_hz, data_lidar_delay, data_lidar_bw = cargar_datos("lidar")

# --- Función para graficar ---
def plot_metric(data, title, ylabel, filename, marker='o', linestyle='--'):
    plt.figure(figsize=(10, 5))
    all_vals = [v for vals in data.values() for v in vals]
    if all_vals:
        mn, mx = min(all_vals), max(all_vals)
        margin = abs(0.1 * max(abs(mn), abs(mx)))
        plt.ylim(mn - margin, mx + margin)
    for escenario, vals in data.items():
        if not vals:
            continue
        x = list(range(len(vals)))
        plt.plot(x, vals, label=escenario.replace('_', ' '), marker=marker, linestyle=linestyle, markersize=3)
        for i, v in enumerate(vals):
            if v == 0:
                plt.plot(i, 0, 'ro')
    plt.title(title)
    plt.xlabel("Muestra")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

# --- Graficar image ---
plot_metric(data_image_hz, "Frecuencia - image topic", "Hz", "frecuencia_image.png")
plot_metric(data_image_delay, "Delay - image topic", "s", "delay_image.png", marker='x')
plot_metric(data_image_bw, "Ancho de banda - image topic", "MB/s", "ancho_banda_image.png", marker='x')

# --- Graficar lidar ---
plot_metric(data_lidar_hz, "Frecuencia - lidar topic", "Hz", "frecuencia_lidar.png")
plot_metric(data_lidar_delay, "Delay - lidar topic", "s", "delay_lidar.png", marker='x')
plot_metric(data_lidar_bw, "Ancho de banda - lidar topic", "MB/s", "ancho_banda_lidar.png", marker='x')
