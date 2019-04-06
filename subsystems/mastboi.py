import enum
import logging

import wpilib
from wpilib.command.subsystem import Subsystem
from wpilib.command import Command


class MastBoi(Subsystem):
    """
    This subsystem controls the Mast with a Neo motor.
    """
    def __init__(self):
        super().__init__("MastBoi")
        self.logger = logging.getLogger("MastBoi")
        self.mastMotor = wpilib.Talon(1)

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

    class HoistTheColors(Command):

        def __init__(self, mastboi):
            super().__init__("Hoist the Colors")
            self.logger = logging.getLogger("Hoist the Colors")
            self.requires(mastboi)
            self._mastboi = mastboi

        def isFinished(self):
            return False

        def initialize(self):
            self.logger.info("initialize not implemented yet")

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
            self.logger.info("initialized not implemented yet")

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
