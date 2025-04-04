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
    for x in range(0, W):
        for y in range(0, H):
            b = camera.imageGetBlue(image, W, x, y)
            g = camera.imageGetGreen(image, W, x, y)
            r = camera.imageGetRed(image, W, x, y)

            # TODO: Procesar el pixel (x,y) de la imagen.
            # ...
    
"""
Se acumulan menos errores, si se realizan movimientos discretos celda a
celda (el robot avanza o gira, se detiene, sensoriza, 
vuelve a avanzar o girar).
"""
def wallFollowing(left_sensor, front_sensor, leftWheel, rightWheel, posL, posR, robot):
    left_wall = left_sensor > 150
    front_wall = front_sensor > 150
        
    left_speed = CRUISE_SPEED
    right_speed = CRUISE_SPEED

    leftWheel.setVelocity(left_speed)
    rightWheel.setVelocity(right_speed)
  
    if front_wall:
        print("Girando derecha")
        deltaL = np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
        deltaR = -deltaL
        giro_90L = posL.getValue() + deltaL
        giro_90R = posR.getValue() + deltaR
        leftWheel.setPosition(giro_90L)
        rightWheel.setPosition(giro_90R)
        while(robot.step(TIME_STEP) != -1 and (posL.getValue() < giro_90L-np.pi/90 or posR.getValue() < giro_90R-np.pi/90)):
            continue
    else:
        if left_wall:
            print("Yendo recto")
            deltaL = 250 / RADIO
            deltaR = deltaL
            avanzeL = posL.getValue() + deltaL
            avanzeR = posR.getValue() + deltaR
            leftWheel.setPosition(avanzeL)
            rightWheel.setPosition(avanzeR)
            while(robot.step(TIME_STEP) != -1 and (posL.getValue() < avanzeL or posR.getValue() < avanzeR)):
                continue
        else:
            print("Girando izquierda")
            deltaR = np.pi/2 * RADIO_ENTRE_RUEDAS / RADIO
            deltaL = -deltaR
            giro_90L = posL.getValue() + deltaL
            giro_90R = posR.getValue() + deltaR
            leftWheel.setPosition(giro_90L)
            rightWheel.setPosition(giro_90R)
            while(robot.step(TIME_STEP) != -1 and (posL.getValue() <= giro_90L-np.pi/90 or posR.getValue() <= giro_90R-np.pi/90)):
                continue
            # y luego ir recto
            deltaL = 250 / RADIO
            deltaR = deltaL
            avanzeL = posL.getValue() + deltaL
            avanzeR = posR.getValue() + deltaR
            leftWheel.setPosition(avanzeL)
            rightWheel.setPosition(avanzeR)
            while(robot.step(TIME_STEP) != -1 and (posL.getValue() < avanzeL or posR.getValue() < avanzeR)):
                continue
        time.sleep(SLEEP)


def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, leftWheel, rightWheel, irSensorList, posL, posR, camera = init_devices(TIME_STEP)
    
    print(irSensorList)
    # Ejecutamos una sincronización para tener disponible el primer frame de la cámara.
    robot.step(TIME_STEP)
    
    for sensor in irSensorList:
        print(sensor.getValue())
    
    # TODO Implementar arquitectura de control del robot.
    # 1 etapa: Wall following y mapeado
    
    while(robot.step(TIME_STEP) != -1):
        # Cogemos valores de los sensores
        left_sensor = irSensorList[1].getValue()
        front_sensor = irSensorList[3].getValue() 

        wallFollowing(left_sensor, front_sensor, leftWheel, rightWheel, posL, posR, robot)
        print("Mapeo")
        
    # 2 etapa: Patrullar y volver a base


if __name__ == "__main__":
    main()
