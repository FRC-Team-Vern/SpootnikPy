from wpilib.joystick import Joystick
from wpilib.buttons.joystickbutton import JoystickButton


class OI(object):

    def __init__(self):
        # Create Joysticks
        self._driveJoy = Joystick(0)
        self._cyController = Joystick(1)

        # Create Buttons
        self._moveMastUpButton = JoystickButton(self._cyController, 8)
        self._moveMastDownButton = JoystickButton(self._cyController, 9)

        # Connect Buttons to Commands
        # self._moveMastUpButton.whileHeld(Robot.mastyBoi.HoistTheColors())
        # self._moveMastDownButton.whileHeld(Robot.mastyBoi.RetreiveTheColors())

    @property
    def driveJoy(self):
        return self._driveJoy


# def getJoystick():
#     """
#     Assign commands to button actions, and publish your joysticks so you
#     can read values from them later.
#     """
#
#     cyController = Joystick(1)
#
#     trigger = JoystickButton(joystick, Joystick.ButtonType.kTrigger)
#     trigger.whenPressed(Crash())
#
# return joystick
