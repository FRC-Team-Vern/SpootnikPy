import logging

from wpilib.command.pidsubsystem import PIDSubsystem
from robotmap import RobotMap as RM
from wpilib.drive import MecanumDrive
from ctre.wpi_talonsrx import WPI_TalonSRX
from ctre._impl.autogen.ctre_sim_enums import NeutralMode
from wpilib.interfaces.generichid import GenericHID
from wpilib.command import Command, TimedCommand


class SpootnikDrives(PIDSubsystem):

    def __init__(self):
        super().__init__(p=0.015, i=0.0001, d=0.0)
        self._deadband = 0.1
        self._turnOutput = 0.0

        # Configure motors
        motors = [WPI_TalonSRX(i) for i in [RM.DRIVE_LF, RM.DRIVE_LB, RM.DRIVE_RF, RM.DRIVE_RB]]
        for i, motor in enumerate(motors):
            # motor.configFactoryDefault()
            motor.enableVoltageCompensation(True)
            motor.configOpenLoopRamp(1.4, 10)
            motor.setNeutralMode(NeutralMode.Brake)

            # Invert left side motors?
            # if i <= 1:
            #     motor.setInverted(True)

            # Set up PIDSubsystem parameters
            self.setInputRange(0.0, 360.0)
            self.pidController = self.getPIDController()
            self.pidController.setContinuous(True)
            self.pidController.setAbsoluteTolerance(1.0)
            self.setSetpoint(0.0)
            # Enable this is you use the PID functionality
            # self.pidController.enable()

        self.drive = MecanumDrive(*motors)
        self.drive.setExpiration(1)
        self.drive.setSafetyEnabled(False)
        self.drive.setDeadband(0.1)

    def returnPIDInput(self):
        return 0.0

    def usePIDOutput(self, output: float):
        self._turnOutput = output

    def initDefaultCommand(self):
        self.setDefaultCommand(self.DriveWithJoy(self))

# Define commands

    class DriveWithJoy(Command):

        def __init__(self, spootnikDrives):
            super().__init__("DriveWithJoy")
            self.logger = logging.getLogger("DriveWithJoy")
            self.requires(spootnikDrives)
            self._spootnikDrives = spootnikDrives
            self._driveSwitcherVal = Command.getRobot().driveSwitcher.getSelected()
            self._printDriveType()

        def execute(self):
            if self._driveSwitcherVal == Command.getRobot().DriveSwitcher.WPI_MECANUM:
                self._spootnikDrives.driveCartesianWithJoy()
            elif self._driveSwitcherVal == Command.getRobot().DriveSwitcher.MORPHEUS:
                self._spootnikDrives.morpheusDrive()
            elif self._driveSwitcherVal == Command.getRobot().DriveSwitcher.FIELD_ORIENTED:
                self._spootnikDrives.fieldOrientedDrive()
            elif self._driveSwitcherVal == Command.getRobot().DriveSwitcher.NO_JOY:
                self._spootnikDrives.noJoyDrive()

        def isFinished(self):
            return False

        def _printDriveType(self):
            if self._driveSwitcherVal == Command.getRobot().DriveSwitcher.WPI_MECANUM:
                self.logger.info("WPI Mecanum Drive selected")
            elif self._driveSwitcherVal == Command.getRobot().DriveSwitcher.MORPHEUS:
                self.logger.info("Morpheus Drive selected")
            elif self._driveSwitcherVal == Command.getRobot().DriveSwitcher.FIELD_ORIENTED:
                self.logger.info("Field Oriented Drive selected")
            elif self._driveSwitcherVal == Command.getRobot().DriveSwitcher.NO_JOY:
                self.logger.info("No Joy Drive selected")

    class DriveForTime(TimedCommand):

        def __init__(self, spootnikDrives, time):
            super().__init__(name="DriveForTime", timeoutInSeconds=time, subsystem=spootnikDrives)
            self.logger = logging.getLogger("DriveForTime")
            self._spootnikDrives = spootnikDrives
            self._angle = 0.0

        def initialize(self):
            self.logger.info("initialize")
            # self._spootnikDrives.drivelyMoreBetterer(0.8, 0.0, 0.0, 0.0)
            self._spootnikDrives.drive.driveCartesian(0.0, 0.8, 0.0)

        def end(self):
            self.logger.info("end")
            self._spootnikDrives.drive.driveCartesian(0.0, 0.0, 0.0)

# Define support functions

    def driveCartesianWithJoy(self):
        joy = Command.getRobot().oi.driveJoy
        ySpeed = joy.getX(GenericHID.Hand.kLeft)
        xSpeed = -joy.getY(GenericHID.Hand.kLeft)
        zRotation = joy.getX(GenericHID.Hand.kRight)
        self.drive.driveCartesian(ySpeed, xSpeed, zRotation)

    def morpheusDrive(self):
        print("INFO: morpheusDrive currently unimplemented")

    def fieldOrientedDrive(self):
        # lx = Command.getRobot().jml.getLX()
        # ly = -Command.getRobot().jml.getLY()
        # rx = Command.getRobot().jml.getRX()

        lx = 0.0
        ly = 0.0
        rx = 0.0

        lx = self.applyDeadband(lx)
        ly = self.applyDeadband(ly)
        rx = self.applyDeadband(rx)

        self.drivelyMoreBetterer(ly, lx, rx)

    def noJoyDrive(self):
        print("INFO: noJoyDrive currently unimplemented")

    def drivelyMoreBetterer(self, ySpeed, xSpeed, zRotation, gyroAngle=0.0):
        pass
        # print("INFO: drivelyMoreBetterer currently unimplemented")

    def applyDeadband(self, val):
        return val if abs(val) > self._deadband else 0.0
