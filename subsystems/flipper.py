import logging
import enum

from wpilib.command.subsystem import Subsystem
from robotmap import RobotMap as RM
from rev import CANSparkMax, MotorType
from wpilib.command import Command


class Flipper(Subsystem):

    FLIPPER_HIGH = -95.0
    FLIPPER_LOW = 0.0

    def __init__(self):
        super().__init__()
        self.flippy_motor = CANSparkMax(RM.FLIPPY_MOTOR, MotorType.kBrushless)
        self.flippy_motor.restoreFactoryDefaults()
        self.encoder = self.flippy_motor.getEncoder()

    def initDefaultCommand(self):
        self.setDefaultCommand(self.Stopification(self))

    class Stopification(Command):

        def __init__(self, flipper):
            super().__init__(subsystem=flipper)
            self._flipper = flipper

        def initialize(self):
            self._flipper.flippy_motor.set(0.0)

        def isFinished(self):
            return False

    class Smash(Command):

        def __init__(self, flipper, flip_direction):
            super().__init__(subsystem=flipper)
            self._flipper = flipper
            self._flip_direction = flip_direction
            self._cycle_counter = 0

        def isFinished(self):
            return False

        def execute(self):
            position = self._flipper.encoder.getPosition()
            if self._flip_direction == Flipper.FlipDirection.UP:
                if position > Flipper.FLIPPER_HIGH:
                    self._flipper.flippy_motor.set(-1.0)
                else:
                    self._flipper.flippy_motor.set(0.0)

            elif self._flip_direction == Flipper.FlipDirection.DOWN:
                if position < Flipper.FLIPPER_LOW:
                    self._flipper.flippy_motor.set(0.5)
                else:
                    self._flipper.flippy_motor.set(0.0)

        def interrupted(self):
            self._flipper.flippy_motor.set(0.0)
            self._cycle_counter = 0

        def end(self):
            self._flipper.flippy_motor.set(0.0)
            self._cycle_counter = 0

    class FlipDirection(enum.IntEnum):
        UP = 0
        DOWN = 1
