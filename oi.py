from wpilib import XboxController
from wpilib.joystick import Joystick
from wpilib.buttons.joystickbutton import JoystickButton
from wpilib.command import Command


class OI(object):

    def __init__(self):
        # Create Joysticks
        self._driveJoy = XboxController(0)
        self._cyController = Joystick(1)

        # Create Buttons
        # self._moveMastUpButton = JoystickButton(self._cyController, 8)
        # self._moveMastDownButton = JoystickButton(self._cyController, 9)

        # Testing
        self._moveMastUpButton = JoystickButton(self._driveJoy, 1)
        self._moveMastDownButton = JoystickButton(self._driveJoy, 2)

        # Connect Buttons to Commands
        mastyBoi = Command.getRobot().mastyBoi
        self._moveMastUpButton.whileHeld(mastyBoi.HoistTheColors(mastyBoi))
        self._moveMastDownButton.whileHeld(mastyBoi.RetrieveTheColors(mastyBoi))

    @property
    def driveJoy(self):
        return self._driveJoy
