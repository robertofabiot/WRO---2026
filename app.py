from robot import *

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

izquierda = Motor(Port.A, Direction.COUNTERCLOCKWISE)
derecha = Motor(Port.B, Direction.CLOCKWISE)

robot = Robot(izquierda, derecha)

robot.avanzar_cm(20)