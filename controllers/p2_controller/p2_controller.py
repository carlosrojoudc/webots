"""
Robótica
Grado en Ingeniería Informática
Universidade da Coruña
Author: Alejandro Paz

Ejemplo de uso de sensores/actuadores básicos y cámara con robot Khepera4 en Webots.
"""

from controller import Robot  # Módulo de Webots para el control el robot.
from controller import Camera  # Módulo de Webots para el control de la cámara.

import time  # Si queremos utilizar time.sleep().
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import cv2  # Si queremos utilizar OpenCV para procesar la imagen.
import sys
from pathfinding import *

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32
# Datos odometria
# Espacio entre ruedas
WHEEL_SPACING = 108.29
# Radio
RADIO = 21
RADIO_ENTRE_RUEDAS = WHEEL_SPACING /2
TAMANO_CELDA = 250
# Time delay between fucntions
SLEEP = 0.05
# Tamaño del mapa
MAP_SIZE = (25,25)
# Error minimo giros ()
ERROR = np.pi/180
# Umbral para deteccion de pared
UMBRAL = 175
# Umbral para la deteccion de amarillo
THRESHOLD = 0.1

# Nombres de los sensores de distancia basados en infrarrojo.
INFRARED_SENSORS_NAMES = [
    "rear left infrared sensor",
    "left infrared sensor",
    "front left infrared sensor",
    "front infrared sensor",
    "front right infrared sensor",
    "right infrared sensor",
    "rear right infrared sensor",
    "rear infrared sensor",
]

GRID_OCUPACION = [[]]

def enable_distance_sensors(robot, timeStep, sensorNames):
    """
    Obtener y activar los sensores de distancia.

    Return: lista con los sensores de distancia activados, en el mismo orden
    establecido en la lista de  nombres (sensorNames).
    """

    sensorList = []

    for name in sensorNames:
        sensorList.append(robot.getDevice(name))

    for sensor in sensorList:
        sensor.enable(timeStep)

    return sensorList


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # Get pointer to the robot.
    robot = Robot()

    # Si queremos obtener el timestep de la simulación.
    # simTimeStep = int(robot.getBasicTimeStep())

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    irSensorList = enable_distance_sensors(robot, timeStep, INFRARED_SENSORS_NAMES)

    # Obtener el dispositivo de la cámara
    camera = robot.getDevice("camera")
    # Activar el dispositivo de la cámara (el tiempo de actualización de los frames
    # de la cámara no debería ser muy alto debido al alto volumen de datos que implica).
    camera.enable(timeStep * 10)

    # Obtener y activar los sensores de posición de las ruedas (encoders).
    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timeStep)
    posR.enable(timeStep)

    # TODO: Obtener y activar otros dispositivos necesarios.
    # ...

    return robot, leftWheel, rightWheel, irSensorList, posL, posR, camera


def process_image_rgb(camera):
    """
    Procesamiento del último frame capturado por el dispositivo de la cámara
    (según el time_step establecido para la cámara).
    ¡ATENCIÓN!: Esta función no es thread-safe, ya que accede al buffer en memoria de la cámara.

    RECOMENDACIÓN: utilizar OpenCV para procesar más eficientemente la imagen
    (ej. hacer una detección de color en HSV).
    """

    W = camera.getWidth()
    H = camera.getHeight()

    image = camera.getImage()

    # Si es suficiente, podríamos procesar solo una parte de la imagen para optimizar .
    for x in range(W//2, W//2+1):
        for y in range(H//2, H//2+1):
            b = camera.imageGetBlue(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            r = camera.imageGetRed(image, W, x, y)

            print(f"{b},{g},{r}")
            # TODO: Procesar el pixel (x,y) de la imagen.
            # ...
    
"""
Se acumulan menos errores, si se realizan movimientos discretos celda a
celda (el robot avanza o gira, se detiene, sensoriza, 
vuelve a avanzar o girar).
"""
def wallFollowing(sensors, leftWheel, rightWheel, posL, posR, robot, robot_pos, direction):
    direction_deltas = [
        (-1, 0),   # North
        (0, 1),   # East
        (1, 0),  # South
        (0, -1)    # West
    ]
    
    # Lectura de sensores
    left_sensor = sensors[1].getValue()
    front_sensor = sensors[3].getValue()

    left_wall = left_sensor > 150
    front_wall = front_sensor > 150


    if front_wall:
        direction = (direction+1)%4
        deltaL = np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
        deltaR = -deltaL
        giro_90L = posL.getValue() + deltaL
        giro_90R = posR.getValue() + deltaR
        leftWheel.setPosition(giro_90L)
        rightWheel.setPosition(giro_90R)
        while(robot.step(TIME_STEP) != -1 and (posL.getValue() < giro_90L-ERROR or posR.getValue() < giro_90R-ERROR)):
            continue
    else:
        if left_wall:
            deltaL = 250 / RADIO
            deltaR = deltaL
            avanzeL = posL.getValue() + deltaL
            avanzeR = posR.getValue() + deltaR
            leftWheel.setPosition(avanzeL)
            rightWheel.setPosition(avanzeR)
            while(robot.step(TIME_STEP) != -1 and (posL.getValue() < avanzeL or posR.getValue() < avanzeR)):
                continue
            # Actualizamos posicion del robot cada vez que avanzamos
            x,y = robot_pos
            dx,dy = direction_deltas[direction]
            robot_pos = (x+dx, y+dy)
        else:
            # Restamos cada vez que giramos a la izquierda
            direction = (direction-1) %4
            deltaR = np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
            deltaL = -deltaR
            giro_90L = posL.getValue() + deltaL
            giro_90R = posR.getValue() + deltaR
            leftWheel.setPosition(giro_90L)
            rightWheel.setPosition(giro_90R)
            while(robot.step(TIME_STEP) != -1 and (posL.getValue() <= giro_90L-ERROR or posR.getValue() <= giro_90R-ERROR)):
                continue
            # Y avanzamos
            deltaL = 250 / RADIO
            deltaR = deltaL
            avanzeL = posL.getValue() + deltaL
            avanzeR = posR.getValue() + deltaR
            leftWheel.setPosition(avanzeL)
            rightWheel.setPosition(avanzeR)
            while(robot.step(TIME_STEP) != -1 and (posL.getValue() < avanzeL or posR.getValue() < avanzeR)):
                continue
            # Actualizamos posicion del robot cada vez que avanzamos
            x,y = robot_pos
            dx,dy = direction_deltas[direction]
            robot_pos = (x+dx, y+dy)

    return robot_pos, direction

def mapping(mapa, robot_position, direction, sensors):

    leftW = sensors[1].getValue() > 175
    frontW = sensors[3].getValue() > 175
    #rightW = sensors[5].getValue() > 150

    x, y = robot_position

    match direction:
        case 0:
            if leftW:
                mapa[x, y-1] = 1
            #if frontW:
                #mapa[x-1, y] = 1
        case 1:
            if leftW:
                mapa[x-1, y] = 1
            if frontW:
                mapa[x, y+1] = 1
        case 2:
            if leftW:
                mapa[x, y+1] = 1
            if frontW:
                mapa[x+1, y] = 1
        case 3:
            if leftW:
                mapa[x+1, y] = 1
            if frontW:
                mapa[x, y-1] = 1
        case _:
            print(f"ERROR DIRECCION: {direction}")
    return mapa

"""
    El robot si empieza con una unica pared a su derecha, gira a la izquierda
    y avanza lo que provoca un comportamiento erroneo.
    Init_position deja al robot con la pared a su izquierda.
"""
def init_position(sensors, robot, leftW, rightW, posL, posR):
    if sensors[1].getValue() > 150 or sensors[3].getValue() > 150:
        return
    if sensors[5].getValue() > 150:
        deltaL = np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
        deltaR = -deltaL
        giro_90L = posL.getValue() + deltaL
        giro_90R = posR.getValue() + deltaR
        leftW.setPosition(giro_90L)
        rightW.setPosition(giro_90R)
        while(robot.step(TIME_STEP) != -1 and (posL.getValue() < giro_90L-ERROR or posR.getValue() < giro_90R-ERROR)):
            continue

def optimized_map(mapa, init_pos):
    x,y = init_pos
    rows = np.any(mapa == 1, axis=1)
    cols = np.any(mapa == 1, axis=0)

    if not np.any(rows) or not np.any(cols):
        return "MAPA VACIO"

    first_row = np.argmax(rows)
    last_row = len(rows) - np.argmax(rows[::-1])
    first_col = np.argmax(cols)
    last_col = len(cols) - np.argmax(cols[::-1])

    new_map = mapa[first_row:last_row, first_col:last_col]
    x =  x-first_row
    y = y-first_col
    return new_map, (x,y)

"""
    Comportamiento de retorno a base
"""
def base(mapa, robot_pos, init_pos):
    path = find_path(mapa, robot_pos, init_pos)
    print(path)
    return path
    

"""
    Check ratio of yellow pixels 
"""
def check_camera(camera):
    W = camera.getWidth()
    H = camera.getHeight()

    image = np.array(camera.getImageArray(), np.uint8)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
    # define range of yellow color in HSV
    lower_yellow = np.array([90,80,80])
    upper_yellow = np.array([100,255,255])

    #print(hsv[W//2][H//2])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    size = cv2.countNonZero(mask)

    current = size/(W*H)
    if current >= THRESHOLD:
        return True
    return False


def get_direction(robot_pos, obj_pos):
    x,y = robot_pos
    ox,oy = obj_pos
    if x < ox:
        if y == oy:
            return 2
    else:
        if y==oy:
            return 0
        elif y < oy:
            return 1
        else:
            return 3
        

def move_robot(leftWheel, rightWheel, posL, posR, robot, robot_pos, direction, desired_direction):
    grados = 0
    diference = abs(direction -desired_direction)
    # Calculamos la direccion a la que tiene que girar
    if diference == 0:
        # No tiene que girar
        grados = 0
    elif diference > 2:
        if direction < desired_direction:
            # Tiene que girar a izquierda
            direction = (direction-1)%4
            grados = -np.pi/2
        else:
            # Tiene que girar a derecha
            direction = (direction+1)%4
            grados = np.pi/2
    elif diference == 2:
        direction = (direction+2)%4
        # Tiene que girar 180 grados
        grados = np.pi
    else:
        if direction < desired_direction:
            # Tiene que girar a derecha
            direction = (direction+1)%4
            grados = np.pi/2
        else:
            # Tiene que girar a izquierda
            direction = (direction-1)%4
            grados = -np.pi/2
    
    # Gira:
    deltaL = grados * RADIO_ENTRE_RUEDAS / RADIO
    deltaR = -deltaL
    giro_90L = posL.getValue() + deltaL
    giro_90R = posR.getValue() + deltaR
    leftWheel.setPosition(giro_90L)
    rightWheel.setPosition(giro_90R)
    while(robot.step(TIME_STEP) != -1 and (posL.getValue() < giro_90L-ERROR or posR.getValue() < giro_90R-ERROR)):
        continue

    direction_deltas = [
        (-1, 0),   # North
        (0, 1),   # East
        (1, 0),  # South
        (0, -1)    # West
    ]
    
    # Avanza:
    deltaL = 250 / RADIO
    deltaR = deltaL
    avanzeL = posL.getValue() + deltaL
    avanzeR = posR.getValue() + deltaR
    leftWheel.setPosition(avanzeL)
    rightWheel.setPosition(avanzeR)
    while(robot.step(TIME_STEP) != -1 and (posL.getValue() < avanzeL or posR.getValue() < avanzeR)):
        continue
    # Actualizamos posicion del robot cada vez que avanzamos
    x,y = robot_pos
    dx,dy = direction_deltas[direction]
    robot_pos = (x+dx, y+dy)

    return robot_pos, direction



def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, leftWheel, rightWheel, irSensorList, posL, posR, camera = init_devices(TIME_STEP)

    # Ejecutamos una sincronización para tener disponible el primer frame de la cámara.
    robot.step(TIME_STEP)

    # Establecemos la velocidad del robot
    left_speed = CRUISE_SPEED
    right_speed = CRUISE_SPEED
    leftWheel.setVelocity(left_speed)
    rightWheel.setVelocity(right_speed)
    
    # TODO Implementar arquitectura de control del robot.
    # 1 etapa: Wall following y mapeado
    x,y = MAP_SIZE
    initial_pos = (x//2, y//2)
    robot_pos = initial_pos
    dir = 0

    map = np.zeros(MAP_SIZE, np.uint8)

    # Colocar el robot con el muro a su izquierda, delante o detras para evitar errores
    init_position(irSensorList, robot, leftWheel, rightWheel, posL, posR)

    # Variable que controla si nos movemos por primera vez
    # De esta forma controlamos la condicion de parada
    init_move = -2
    while(robot.step(TIME_STEP) != -1):
        robot_pos, dir = wallFollowing(irSensorList, leftWheel, rightWheel, posL, posR, robot, robot_pos, dir)
        time.sleep(SLEEP)
        map = mapping(map, robot_pos, dir, irSensorList)
        # Llegamos al principio?
        if (initial_pos == robot_pos and (init_move >= 0)):
            map, initial_pos = optimized_map(map, initial_pos)
            print(map)
            break
        init_move += 1
    

    # 2 ETAPA: Exploracion y reconocimiento
    robot_pos = initial_pos
    objeto_interes = False
    time.sleep(SLEEP)

    while(robot.step(TIME_STEP) != -1):
        robot_pos, dir = wallFollowing(irSensorList, leftWheel, rightWheel, posL, posR, robot, robot_pos, dir)
        objeto_interes = check_camera(camera)
        if (objeto_interes):
            ruta = base(map, robot_pos, initial_pos)
            if ruta:
                break

    print(dir)        
    time.sleep(SLEEP)
    print(robot_pos)
    # Vuelta a base
    for x,y in ruta:
        rx,ry = robot_pos
        if x == rx and y == ry:
            continue
        print(f"x: {x}, y: {y}")
        desired_direction = get_direction(robot_pos, (x,y))
        robot_pos, dir = move_robot(leftWheel, rightWheel, posL, posR, robot, robot_pos, dir, desired_direction)



if __name__ == "__main__":
    main()
