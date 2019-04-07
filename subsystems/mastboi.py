import enum
import logging

from rev import CANSparkMax
from rev._impl.autogen.sim_enums import MotorType, ControlType
from wpilib.command.subsystem import Subsystem
from wpilib.command import Command

from robotmap import RobotMap as RM


class MastBoi(Subsystem):
    """
    This subsystem controls the Mast with a Neo motor.
    """
    BOTTOM_ENCODER_VALUE = 0.0
    MIDDLE_ENCODER_VALUE = -68.0
    TOP_ENCODER_VALUE = -136.0

    def __init__(self):
        super().__init__("MastBoi")
        self.logger = logging.getLogger("MastBoi")
        self.mastMotor = CANSparkMax(RM.MAST, MotorType.kBrushless)
        self.encoder = self.mastMotor.getEncoder()
        self.encoder.setPosition(0.0)

    def initDefaultCommand(self):
        self.logger.warning("No default command for MastBoi")

# Define commands

    class Stop(Command):

        def __init__(self, mastboi):
            super().__init__("Stop")
            self.logger = logging.getLogger("Stop")
            self.requires(mastboi)
            self._mastboi = mastboi

        def isFinished(self):
            return False

        def initialize(self):
            self.logger.info("initialize")
            self._mastboi.mastMotor.set(0.0)

    class HoistTheColorsToPosition(Command):

        def __init__(self, mastboi, mastPosition):
            super().__init__("Hoist the Colors To Position")
            self.logger = logging.getLogger("Hoist the Colors To Position")
            self.requires(mastboi)
            self._mastboi = mastboi
            self.mastPosition = mastPosition

        def isFinished(self):
            return False

        def initialize(self):
            self.logger.info("initialize")
            setPoint = 0
            if self.mastPosition == self._mastboi.MastPosition.BOTTOM:
                setPoint = MastBoi.BOTTOM_ENCODER_VALUE
            elif self.mastPosition == self._mastboi.MastPosition.MIDDLE:
                setPoint = MastBoi.MIDDLE_ENCODER_VALUE
            elif self.mastPosition == self._mastboi.MastPosition.TOP:
                setPoint = MastBoi.TOP_ENCODER_VALUE
            else:
                self.logger.warning("Unknown mast position")

            self.logger.warning("kSmartMotion not enabled: setPoint: {}".format(setPoint))
            # self._mastboi.mastMotor.set(ControlType.kSmartMotion, setPoint)

        def end(self):
            self.logger.info("end")
            self._mastboi.mastMotor.set(0.0)

    class HoistTheColors(Command):

        def __init__(self, mastboi):
            super().__init__("Hoist the Colors")
            self.logger = logging.getLogger("Hoist the Colors")
            self.requires(mastboi)
            self._mastboi = mastboi

        def isFinished(self):
            return False

        def initialize(self):
            self.logger.info("initialize")

        def execute(self):
            power = 0.6
            position = self._mastboi.encoder.getPosition()
            self.logger.info("position: {}".format(position))
            if position > MastBoi.TOP_ENCODER_VALUE:
                self._mastboi.mastMotor.set(power)
            else:
                self._mastboi.mastMotor.set(0.0)

        def end(self):
            self.logger.info("end")
            self._mastboi.mastMotor.set(0.0)

    class RetrieveTheColors(Command):

        def __init__(self, mastboi):
            super().__init__("Retrieve the Colors")
            self.logger = logging.getLogger("Retrieve the Colors")
            self.requires(mastboi)
            self._mastboi = mastboi

        def isFinished(self):
            return False

        def initialize(self):
            self.logger.info("initialized")

        def execute(self):
            power = 0.1
            position = self._mastboi.encoder.getPosition()
            if position < MastBoi.BOTTOM_ENCODER_VALUE:
                self._mastboi.mastMotor.set(power)
            else:
                self._mastboi.mastMotor.set(0.0)

        def end(self):
            self.logger.info("end")
            self._mastboi.mastMotor.set(0.0)

# Define Enums

    class MastPosition(enum.IntEnum):
        BOTTOM = 0
        MIDDLE = 1
        TOP = 2

    class MastDirection(enum.IntEnum):
        DOWN = 0
        UP = 1
