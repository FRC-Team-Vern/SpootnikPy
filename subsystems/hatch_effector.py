from wpilib.command.subsystem import Subsystem
from wpilib.command.command import Command
from wpilib.command.instantcommand import InstantCommand
from wpilib.doublesolenoid import DoubleSolenoid
from wpilib.servo import Servo


class HatchEffector(Subsystem):

    def __init__(self):
        super().__init__()
        self.solenoid = DoubleSolenoid(0, 7)
        self.servo = Servo(0)

    class ServoOpen(InstantCommand):
        def __init__(self, hatch_effector):
            super().__init__(subsystem=hatch_effector)
            self._hatch_effector = hatch_effector

        def initialize(self):
            self._hatch_effector.servo.set(0.5)

    class ServoHalf(InstantCommand):
        def __init__(self, hatch_effector):
            super().__init__(subsystem=hatch_effector)
            self._hatch_effector = hatch_effector

        def initialize(self):
            self._hatch_effector.servo.set(0.25)

    class ServoClose(InstantCommand):
        def __init__(self, hatch_effector):
            super().__init__(subsystem=hatch_effector)
            self._hatch_effector = hatch_effector

        def initialize(self):
            self._hatch_effector.servo.set(0.0)

    class ShootThePanel(Command):
        def __init__(self, hatch_effector):
            super().__init__(subsystem=hatch_effector)
            self._hatch_effector = hatch_effector

        def isFinished(self):
            return False

        def initialize(self):
            self._hatch_effector.solenoid.set(DoubleSolenoid.Value.kForward)

        def end(self):
            self._hatch_effector.solenoid.set(DoubleSolenoid.Value.kReverse)