from wpilib.command import Command, CommandGroup


class DriveToBaseLine(CommandGroup):

    def __init__(self):
        super().__init__("Drive to BaseLine")
        spootnikDrives = Command.getRobot().spootnikDrives
        self.addSequential(spootnikDrives.DriveForTime(spootnikDrives, 2.0))
