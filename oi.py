from wpilib import XboxController
from wpilib.joystick import Joystick
from wpilib.buttons.joystickbutton import JoystickButton
from wpilib.command import Command

from subsystems.flipper import Flipper


class OI(object):

    def __init__(self):
        # Create Joysticks
        self._driveJoy = XboxController(0)
        self._cyController = Joystick(1)

        # Create Buttons
        self._blastenTheHatches = JoystickButton(self._driveJoy, 2)
        self._servoOpen = JoystickButton(self._driveJoy, 4)
        self._servoHalf = JoystickButton(self._driveJoy, 3)
        self._servoClose = JoystickButton(self._driveJoy, 1)

        self._suplexButton = JoystickButton(self._driveJoy, 8)
        self._backButton = JoystickButton(self._driveJoy, 7)

        # Testing
        self._moveMastUpButton = JoystickButton(self._cyController, 8)
        self._moveMastDownButton = JoystickButton(self._cyController, 9)

        # Connect Buttons to Commands
        hatchEffector = Command.getRobot().hatchEffector
        self._blastenTheHatches.whileHeld(hatchEffector.ShootThePanel(hatchEffector))
        self._servoOpen.whileHeld(hatchEffector.ServoOpen(hatchEffector))
        self._servoHalf.whileHeld(hatchEffector.ServoHalf(hatchEffector))
        self._servoClose.whileHeld(hatchEffector.ServoClose(hatchEffector))

        suplex = Command.getRobot().suplex
        self._suplexButton.whileHeld(suplex.Smash(suplex, Flipper.FlipDirection.UP))
        self._backButton.whileHeld(suplex.Smash(suplex, Flipper.FlipDirection.DOWN))

        mastyBoi = Command.getRobot().mastyBoi
        self._moveMastUpButton.whileHeld(mastyBoi.HoistTheColors(mastyBoi))
        self._moveMastDownButton.whileHeld(mastyBoi.RetrieveTheColors(mastyBoi))

    @property
    def driveJoy(self):
        return self._driveJoy
