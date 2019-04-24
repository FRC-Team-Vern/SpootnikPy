
import wpilib
import enum

from wpilib.command import Command
from commandbased import CommandBasedRobot

from commands.drivetobaseline import DriveToBaseLine
from subsystems.spootnikdrives import SpootnikDrives
from subsystems.mastboi import MastBoi
from subsystems.flipper import Flipper
from subsystems.cargo_effector import CargoEffector
from oi import OI


class SpootnikRobot(CommandBasedRobot):
    """
    The CommandBasedRobot base class implements almost everything you need for
    a working robot program. All you need to do is set up the subsystems and
    commands. You do not need to override the "periodic" functions, as they
    will automatically call the scheduler. You may override the "init" functions
    if you want to do anything special when the mode changes.
    """
    def robotInit(self):
        """
        This is a good place to set up your subsystems and anything else that
        you will need to access later.
        """
        # Define Robot getter
        Command.getRobot = lambda: self

        # Shuffleboard options
        self.driveSwitcher = wpilib.SendableChooser()
        self.driveSwitcher.setDefaultOption("WPI Mecanum", self.DriveSwitcher.WPI_MECANUM)
        self.driveSwitcher.addOption("Field Oriented", self.DriveSwitcher.FIELD_ORIENTED)
        self.driveSwitcher.addOption("Morpheus", self.DriveSwitcher.MORPHEUS)
        self.driveSwitcher.addOption("No Joystick", self.DriveSwitcher.NO_JOY)

        # Create subsystems
        self.mastyBoi = MastBoi()
        self.suplex = Flipper()
        self.spootnikDrives = SpootnikDrives()
        self.cargoEffector = CargoEffector()

        # Autonomous commands
        self.autonomousCommand = DriveToBaseLine()

        """
        Since OI instantiates commands and commands need access to subsystems,
        OI must be initialized after subsystems.
        """
        self.oi = OI()

    def autonomousInit(self):
        """
        You should call start on your autonomous program here. You can
        instantiate the program here if you like, or in robotInit as in this
        example. You can also use a SendableChooser to have the autonomous
        program chosen from the SmartDashboard.
        """
        self.autonomousCommand.start()

# Define Enums

    class DriveSwitcher(enum.IntEnum):
        MORPHEUS = 0
        FIELD_ORIENTED = 1
        NO_JOY = 2
        WPI_MECANUM = 3


if __name__ == "__main__":
    wpilib.run(SpootnikRobot)
