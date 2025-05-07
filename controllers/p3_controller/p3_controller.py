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

UMBRAL = 200

SLEEP = 0.05

def enable_sensors(robot, timeStep):
    """
    Obtener y activar los sensores del robot.

    Return: lista con los sensores activados.
    """
    
    sensors = {
        "rear left infrared sensor":robot.getDevice("rear left infrared sensor"),
        "left infrared sensor":robot.getDevice("left infrared sensor"),
        "front left infrared sensor":robot.getDevice("front left infrared sensor"),
        "front infrared sensor":robot.getDevice("front infrared sensor"),
        "front right infrared sensor":robot.getDevice("front right infrared sensor"),
        "right infrared sensor":robot.getDevice("right infrared sensor"),
        "rear right infrared sensor":robot.getDevice("rear right infrared sensor"),
        "rear infrared sensor":robot.getDevice("rear infrared sensor"),
        "ground left infrared sensor":robot.getDevice("ground left infrared sensor"),
        "ground front left infrared sensor":robot.getDevice("ground front left infrared sensor"),
        "ground front right infrared sensor":robot.getDevice("ground front right infrared sensor"),
        "ground right infrared sensor":robot.getDevice("ground right infrared sensor")
    }
    
    for sensor in sensors.values():
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
        wheel.setPosition(float('+inf'))
        wheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    sensorList = enable_sensors(robot, timeStep)

    return robot, motores, sensorList

def turn_right( wheels):
    print("Derecha")
    wheels["izquierda"].setVelocity(CRUISE_SPEED*2)
    wheels["derecha"].setVelocity(CRUISE_SPEED)

def turn_left(wheels):
    print("Izquierda")
    wheels["izquierda"].setVelocity(CRUISE_SPEED)
    wheels["derecha"].setVelocity(CRUISE_SPEED*2)


def go_straight(wheels):
    print("Recto")
    wheels["izquierda"].setVelocity(CRUISE_SPEED)
    wheels["derecha"].setVelocity(CRUISE_SPEED)

def refuerzo(sensors, wheels, matrix):

    pass
    #go_straight(wheels)
    #time.sleep(SLEEP)
    #turn_left(wheels)
    #turn_right(wheels)
    #time.sleep(SLEEP + 0.1)


"""
Programar función que determine el estado actual.
"""
def current_state():
    pass

"""
Después de realizar una acción, programar función que establece un valor numérico de
refuerzo (positivo o negativo).
"""
def action_result():
    pass


"""
Actualizar la matriz Q (3x3), con la fórmula de entornos no deterministas
"""
def update_matrix():
    pass


def evitar_paredes(sensors, wheels):
    # simple obstacle avoidance algorithm
    # based on the front infrared sensors
    speed_offset = 0.2 * (MAX_SPEED - 0.03 * sensors["front infrared sensor"].getValue())
    speed_delta = 0.03 * sensors["front left infrared sensor"].getValue() - 0.03 * sensors["front right infrared sensor"].getValue()
    
    wheels["izquierda"].setVelocity(speed_offset + speed_delta)
    wheels["derecha"].setVelocity(speed_offset - speed_delta)

def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, wheels, sensorList = init_devices(TIME_STEP)

    robot.step(TIME_STEP)

    matrizQ = np.zeros((3,3), np.float16)

    while(robot.step(TIME_STEP) != -1):
        if (sensorList["front infrared sensor"].getValue() > UMBRAL or sensorList["front left infrared sensor"].getValue() > UMBRAL or sensorList["front right infrared sensor"].getValue() > UMBRAL):
            evitar_paredes(sensorList, wheels)
        else:
            refuerzo(sensorList, wheels, matrizQ)
            

if __name__ == "__main__":
    main()

"""
Hacer exploracion para refinar Q.
"""