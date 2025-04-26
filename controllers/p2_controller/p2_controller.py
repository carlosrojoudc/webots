"""
Robótica
Grado en Ingeniería Informática
Universidade da Coruña
Author: Alejandro Paz y José Manuel Fernández Montáns

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

GRID_OCUPACION = [[]]

def enable_sensors(robot, timeStep):
    """
    Obtener y activar los sensores del robot.

    Return: lista con los sensores activados.
    """
    
    sensors = {
        "left infrared sensor":robot.getDevice("left infrared sensor"),
        "front infrared sensor":robot.getDevice("front infrared sensor"),
        "right infrared sensor":robot.getDevice("right infrared sensor"),
        "front left infrared sensor":robot.getDevice("front left infrared sensor"),
        "front right infrared sensor":robot.getDevice("front right infrared sensor"),
        "left wheel sensor":robot.getDevice("left wheel sensor"),
        "right wheel sensor":robot.getDevice("right wheel sensor"),
        "camera":robot.getDevice("camera")
    }
    
    for sensor in sensors.values():
        if sensor is sensors["camera"]:
            sensor.enable(timeStep*10)
        else:
            sensor.enable(timeStep)

    return sensors


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # referencia al robot
    robot = Robot()

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    
    motores = {
        "derecha":robot.getDevice("right wheel motor"),
        "izquierda":robot.getDevice("left wheel motor")
    }

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    
    for wheel in motores.values():
        wheel.setPosition(0)
        wheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    sensorList = enable_sensors(robot, timeStep)

    return robot, motores, sensorList

def turn_right(robot, posL, posR, wheels):
    deltaL = np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
    deltaR = -deltaL
    giro_90L = posL.getValue() + deltaL
    giro_90R = posR.getValue() + deltaR
    wheels["izquierda"].setPosition(giro_90L)
    wheels["derecha"].setPosition(giro_90R)
    while(robot.step(TIME_STEP) != -1 and (posL.getValue() < giro_90L-ERROR or posR.getValue() < giro_90R-ERROR)):
        continue
    
def turn_left(robot, posL, posR, wheels):
    deltaL = -np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
    deltaR = -deltaL
    giro_90L = posL.getValue() + deltaL
    giro_90R = posR.getValue() + deltaR
    wheels["izquierda"].setPosition(giro_90L)
    wheels["derecha"].setPosition(giro_90R)
    while(robot.step(TIME_STEP) != -1 and (posL.getValue() < giro_90L-ERROR or posR.getValue() < giro_90R-ERROR)):
        continue
    
def go_straight(robot, posL, posR, wheels):
    deltaL = 250 / RADIO
    deltaR = deltaL
    avanzeL = posL.getValue() + deltaL
    avanzeR = posR.getValue() + deltaR
    wheels["izquierda"].setPosition(avanzeL)
    wheels["derecha"].setPosition(avanzeR)
    while(robot.step(TIME_STEP) != -1 and (posL.getValue() < avanzeL or posR.getValue() < avanzeR)):
        continue
    
"""
Se acumulan menos errores, si se realizan movimientos discretos celda a
celda (el robot avanza o gira, se detiene, sensoriza, 
vuelve a avanzar o girar).
"""
def wallFollowing(sensors, wheels, robot, robot_pos, direction):
    direction_deltas = [
        (-1, 0),   # North
        (0, 1),   # East
        (1, 0),  # South
        (0, -1)    # West
    ]
    
    # Lectura de sensores
    left_sensor = sensors["left infrared sensor"].getValue()
    front_sensor = sensors["front infrared sensor"].getValue()
    left_wall = left_sensor > UMBRAL
    front_wall = front_sensor > UMBRAL


    if front_wall:
        direction = (direction+1)%4
        turn_right(robot, sensors["left wheel sensor"], sensors["right wheel sensor"], wheels)
    else:
        if left_wall:
            go_straight(robot, sensors["left wheel sensor"], sensors["right wheel sensor"], wheels)
        else:
            # Restamos cada vez que giramos a la izquierda
            direction = (direction-1) %4
            turn_left(robot, sensors["left wheel sensor"], sensors["right wheel sensor"], wheels)
            # Y avanzamos
            go_straight(robot, sensors["left wheel sensor"], sensors["right wheel sensor"], wheels)
            
        # Actualizamos posicion del robot cada vez que avanzamos
        x,y = robot_pos
        dx,dy = direction_deltas[direction]
        robot_pos = (x+dx, y+dy)

    return robot_pos, direction

def mapping(mapa, robot_position, direction, sensors):

    leftW = sensors["left infrared sensor"].getValue() > UMBRAL
    rightW = sensors["right infrared sensor"].getValue() > UMBRAL
    frontW = sensors["front infrared sensor"].getValue() > UMBRAL

    x, y = robot_position

    match direction:
        case 0:
            if leftW and mapa[x, y-1] != 2:
                mapa[x, y-1] = 1
            if frontW:
                mapa[x-1, y] = 1 + check_camera(sensors["camera"])
            if rightW and mapa[x, y+1] != 2:
                mapa[x, y+1] = 1
            if leftW and frontW and mapa[x-1, y-1] != 2:
                mapa[x-1, y-1] = 1
            if rightW and frontW and mapa[x-1, y+1] != 2:
                mapa[x-1, y+1] = 1
        case 1:
            if leftW and mapa[x-1, y] != 2:
                mapa[x-1, y] = 1
            if frontW:
                mapa[x, y+1] = 1 + check_camera(sensors["camera"])
            if rightW and mapa[x+1, y] != 2:
                mapa[x+1, y] = 1
            if leftW and frontW and mapa[x-1, y+1] != 2:
                mapa[x-1, y+1] = 1
            if rightW and frontW and mapa[x+1, y+1] != 2:
                mapa[x+1, y+1] = 1
                
        case 2:
            if leftW and mapa[x, y+1] != 2:
                mapa[x, y+1] = 1
            if frontW:
                mapa[x+1, y] = 1 + check_camera(sensors["camera"])
            if rightW and mapa[x, y-1] != 2:
                mapa[x, y-1] = 1
            if leftW and frontW and mapa[x+1, y+1] != 2:
                mapa[x+1, y+1] = 1
            if rightW and frontW and mapa[x+1, y-1] != 2:
                mapa[x+1, y-1] = 1
            
        case 3:
            if leftW and mapa[x+1, y] != 2:
                mapa[x+1, y] = 1
            if frontW:
                mapa[x, y-1] = 1 + check_camera(sensors["camera"])
            if rightW and mapa[x-1, y] != 2:
                mapa[x-1, y] = 1
            if leftW and frontW and mapa[x+1, y-1] != 2:
                mapa[x+1, y-1] = 1
            if rightW and frontW and mapa[x-1, y-1] != 2:
                mapa[x-1, y-1] = 1
        case _:
            print(f"ERROR DIRECCION: {direction}")
    return mapa

"""
    El robot si empieza con una unica pared a su derecha, gira a la izquierda
    y avanza lo que provoca un comportamiento erroneo.
    Init_position deja al robot con la pared a su izquierda.
"""
def init_position(sensors, robot, wheels):
    if sensors["left infrared sensor"].getValue() > UMBRAL or sensors["front infrared sensor"].getValue() > UMBRAL:
        return
    if sensors["right infrared sensor"].getValue() > UMBRAL:
        turn_right(robot, sensors["left wheel sensor"], sensors["right wheel sensor"], wheels)

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
    
    for i, fila in enumerate(new_map):
        for j, casilla in enumerate(fila):
            if casilla == 2:
                new_map[i][j] = 0
    
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
        

def move_robot(wheels, posL, posR, robot, robot_pos, direction, desired_direction):
    diference = abs(direction -desired_direction)
    # Calculamos la direccion a la que tiene que girar
    if diference != 0:
        # No tiene que girar
        if diference > 2:
            if direction < desired_direction:
                # Tiene que girar a izquierda
                direction = (direction-1)%4
                turn_left(robot, posL, posR, wheels)
            else:
                # Tiene que girar a derecha
                direction = (direction+1)%4
                turn_right(robot, posL, posR, wheels)
        elif diference == 2:
            direction = (direction+2)%4
            # Tiene que girar 180 grados
            turn_right(robot, posL, posR, wheels)
            turn_right(robot, posL, posR, wheels)
        else:
            if direction < desired_direction:
                # Tiene que girar a derecha
                direction = (direction+1)%4
                turn_right(robot, posL, posR, wheels)
            else:
                # Tiene que girar a izquierda
                direction = (direction-1)%4
                turn_left(robot, posL, posR, wheels)

    direction_deltas = [
        (-1, 0),   # North
        (0, 1),   # East
        (1, 0),  # South
        (0, -1)    # West
    ]
    
    # Avanza:
    go_straight(robot, posL, posR, wheels)
    # Actualizamos posicion del robot cada vez que avanzamos
    x,y = robot_pos
    dx,dy = direction_deltas[direction]
    robot_pos = (x+dx, y+dy)

    return robot_pos, direction



def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, wheels, sensorList = init_devices(TIME_STEP)

    # Ejecutamos una sincronización para tener disponible el primer frame de la cámara.
    robot.step(TIME_STEP)

    # Establecemos la velocidad del robot
    for wheel in wheels.values():
        wheel.setVelocity(CRUISE_SPEED)
    
    # TODO Implementar arquitectura de control del robot.
    # 1 etapa: Wall following y mapeado
    x,y = MAP_SIZE
    initial_pos = (x//2, y//2)
    robot_pos = initial_pos
    dir = 0

    map = np.zeros(MAP_SIZE, np.uint8)

    # Colocar el robot con el muro a su izquierda, delante o detras para evitar errores
    init_position(sensorList, robot, wheels)

    # Variable que controla si nos movemos por primera vez
    # De esta forma controlamos la condicion de parada
    init_move = -2
    while(robot.step(TIME_STEP) != -1):
        robot_pos, dir = wallFollowing(sensorList, wheels, robot, robot_pos, dir)
        time.sleep(SLEEP)
        map = mapping(map, robot_pos, dir, sensorList)
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
        robot_pos, dir = wallFollowing(sensorList, wheels, robot, robot_pos, dir)
        objeto_interes = check_camera(sensorList["camera"])
        if (objeto_interes):
            ruta = base(map, robot_pos, initial_pos)
            if ruta:
                break
      
    time.sleep(SLEEP)
    # Vuelta a base
    for x,y in ruta:
        rx,ry = robot_pos
        if x == rx and y == ry:
            continue
        print(f"x: {x}, y: {y}")
        desired_direction = get_direction(robot_pos, (x,y))
        robot_pos, dir = move_robot(wheels, sensorList["left wheel sensor"], sensorList["right wheel sensor"], robot, robot_pos, dir, desired_direction)



if __name__ == "__main__":
    main()
