import logging
import enum

from wpilib.command.subsystem import Subsystem
from wpilib.interfaces.generichid import GenericHID
from robotmap import RobotMap as RM
from ctre import WPI_TalonSRX
from wpilib.command import Command


class CargoEffector(Subsystem):

    def __init__(self):
        super().__init__()
        self.cargo_motor = WPI_TalonSRX(RM.CARGO_MOTOR)
        self.cargo_motor.enableVoltageCompensation(True)
        self.cargo_motor.setInverted(True)

    def initDefaultCommand(self):
        self.setDefaultCommand(self.EatTheBallOrNot(self))

    class Stopification(Command):

        def __init__(self, cargo_effector):
            super().__init__(subsystem=cargo_effector)
            self._cargo_effector = cargo_effector

        def isFinished(self):
            return False

        def initialize(self):
            self._cargo_effector.cargo_motor.set(0.0)

    class EatTheBallOrNot(Command):

        def __init__(self, cargo_effector):
            super().__init__(subsystem=cargo_effector)
            self._cargo_effector = cargo_effector
            self._deadband = 0.1
            self.logger = logging.getLogger(self.__class__.__name__)

        def isFinished(self):
            return False

        def execute(self):
            left_trigger = Command.getRobot().oi.driveJoy.getTriggerAxis(GenericHID.Hand.kLeft)
            right_trigger = Command.getRobot().oi.driveJoy.getTriggerAxis(GenericHID.Hand.kRight)

            left_trigger = self.apply_deadband(left_trigger)
            right_trigger = self.apply_deadband(right_trigger)

            # self.logger.info("Left trigger: {}".format(left_trigger))
            # self.logger.info("Right trigger: {}".format(right_trigger))

            if left_trigger > 0:
                self._cargo_effector.cargo_motor.set(left_trigger)
            else:
                self._cargo_effector.cargo_motor.set(-right_trigger)

        def apply_deadband(self, value):
            return value if abs(value) > self._deadband else 0.0

    class EdibleRegurgitation(enum.IntEnum):
        NOMMY = 0
        PEWPEW = 1
