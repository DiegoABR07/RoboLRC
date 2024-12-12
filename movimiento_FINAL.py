from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
import numpy as np
import serial
import time
import math

# Configuración de la comunicación serial
SERIAL_PORT = '/dev/ttyACM0'  # Reemplaza con el puerto serial correcto
BAUD_RATE = 9600  # Debe coincidir con el baud rate del Arduino

# Inicializar la comunicación serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Esperar a que la conexión serial se establezca
    print(f"Puerto serial {SERIAL_PORT} abierto exitosamente.")
except serial.SerialException as e:
    print(f"Error al abrir el puerto serial {SERIAL_PORT}: {e}")
    exit()

# Función para enviar datos por comunicación serial
def enviar_datos_serial(Vx, Vy, omega):
    # Invertir las velocidades para invertir las direcciones
    Vx = -Vx
    Vy = -Vy
    # Convertir los valores a enteros multiplicando por 100 para evitar decimales
    Vx_int = int(Vx * 100)
    Vy_int = int(Vy * 100)
    omega_int = int(omega * 100)
    # Preparar la cadena de datos
    data_str = f"{Vx_int},{Vy_int},{omega_int}\n"
    # Enviar datos por el puerto serial
    ser.write(data_str.encode('utf-8'))
    print(f"Enviado: Vx={Vx}, Vy={Vy}, omega={omega}")

# Configuración de la matriz del entorno (7x7)
matrix = np.array([
    [1, 1, 1, 1, 1, 1, 1],  # Fila 0
    [1, 1, 1, 1, 1, 1, 1],  # Fila 1
    [1, 0, 0, 1, 0, 0, 1],  # Fila 2
    [1, 0, 0, 1, 0, 0, 1],  # Fila 3
    [1, 1, 1, 1, 1, 1, 1],  # Fila 4
    [1, 1, 1, 1, 1, 1, 1],  # Fila 5
    [0, 0, 0, 0, 0, 0, 0],  # Fila 6
])

# Configuración del algoritmo A*
class GridPersonalizado(Grid):
    def nodo_fila_columna(self, fila, columna):
        return self.node(columna, fila)

grid = GridPersonalizado(matrix=matrix)
posicion_actual = (0, 0)  # Posición inicial (fila, columna)
destino_final = (1, 6)    # Posición final (fila, columna)
start = grid.nodo_fila_columna(*posicion_actual)
end = grid.nodo_fila_columna(*destino_final)

finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)
path_coords = [(node.y, node.x) for node in path]
print('Camino encontrado:', path_coords)

# Configuración de parámetros de movimiento
distancia_por_movimiento = 0.3  # 30 cm por movimiento
velocidad = 0.1  # Velocidad de 0.1 m/s (10 cm/s)

# Función para convertir coordenadas en velocidades
def convertir_coordenadas_a_movimiento(coord_actual, coord_siguiente):
    delta_x = coord_siguiente[1] - coord_actual[1]  # Cambio en columna (izquierda/derecha)
    delta_y = coord_siguiente[0] - coord_actual[0]  # Cambio en fila (adelante/atrás)

    # Mapear delta_y a Vx (movimiento adelante/atrás)
    # Mapear delta_x a Vy (movimiento izquierda/derecha)
    if abs(delta_x) == abs(delta_y) and delta_x != 0 and delta_y != 0:  # Movimiento diagonal
        distancia = math.sqrt(2 * distancia_por_movimiento ** 2)
        tiempo_movimiento = distancia / velocidad
        Vx = velocidad * (delta_y / abs(delta_y))
        Vy = velocidad * (delta_x / abs(delta_x))
    else:
        distancia = distancia_por_movimiento
        tiempo_movimiento = distancia / velocidad
        Vx = velocidad * (delta_y / abs(delta_y)) if delta_y != 0 else 0
        Vy = velocidad * (delta_x / abs(delta_x)) if delta_x != 0 else 0

    omega = 0.0  # Sin rotación
    return Vx, Vy, omega, tiempo_movimiento

# Envío de comandos de movimiento por comunicación serial
for i in range(len(path_coords) - 1):
    coord_actual = path_coords[i]
    coord_siguiente = path_coords[i + 1]

    Vx, Vy, omega, tiempo_movimiento = convertir_coordenadas_a_movimiento(coord_actual, coord_siguiente)

    # Enviar los valores por comunicación serial con direcciones invertidas
    enviar_datos_serial(Vx, Vy, omega)

    # Esperar el tiempo necesario para completar el movimiento
    print(f"Moviendo de {coord_actual} a {coord_siguiente}")
    print(f"Vx: {Vx}, Vy: {Vy}, omega: {omega}")
    print(f"Tiempo estimado de movimiento: {tiempo_movimiento:.2f} segundos")
    time.sleep(tiempo_movimiento)

# Enviar valores de detención al finalizar el camino
enviar_datos_serial(0, 0, 0)
print("Movimiento completado y detenido.")

# Cerrar el puerto serial
ser.close()
print(f"Puerto serial {SERIAL_PORT} cerrado.")
