import subprocess
import os
from screeninfo import get_monitors

# Obtener el ancho y la altura de la pantalla
monitor = get_monitors()[0]  # Suponiendo que solo hay un monitor
screen_width = monitor.width
screen_height = monitor.height
docker_command = 'sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6'

# Calcular el tamaño y la posición
width = screen_width // 2  # Por ejemplo, la mitad del ancho
height = screen_height  # Por ejemplo, la mitad de la altura
x_position = (screen_width - width)  # Centrado horizontalmente
y_position = 0  # Centrado verticalmente

# Definir el comando para abrir terminator con geometría
terminator_command = [
    'terminator',
    f'--geometry={width}x{height}+{x_position}+{y_position}',
    '-e', docker_command
]

try:
    # Ejecutar terminator
    subprocess.run(terminator_command, check=True)
except subprocess.CalledProcessError as e:
    print(f"Error al ejecutar terminator: {e}")
