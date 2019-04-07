import pprint

from pyfrc.physics import drivetrains


class PhysicsEngine:

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.drivetrain = drivetrains.MecanumDrivetrain()
        self.initial = True
        self.simulated_position = 0.0

    def update_sim(self, hal_data, now, tm_diff):
        # print(hal_data)
        if self.initial:
            self.initial = False
            pprint.pprint(hal_data["CAN"]["sparkmax-5"])
            # for key in hal_data["sparkmax-5"].keys():
            #     print(key)
        # Simulate the drivetrain
        lf_motor = hal_data["CAN"][4]["value"]
        lr_motor = hal_data["CAN"][1]["value"]
        rf_motor = -hal_data["CAN"][2]["value"]
        rr_motor = -hal_data["CAN"][3]["value"]

        # Update sparkmax-5 position
        current_position = hal_data["CAN"]["sparkmax-5"]["position"]
        if current_position >= 0:
            self.simulated_position += (hal_data["CAN"]["sparkmax-5"]["value"] - 0.2)
            hal_data["CAN"]["sparkmax-5"]["position"] = self.simulated_position
        else:
            self.simulated_position = 0.0
            hal_data["CAN"]["sparkmax-5"]["position"] = self.simulated_position

        xSpeed, ySpeed, rotation = self.drivetrain.get_vector(lr_motor, rr_motor, lf_motor, rf_motor)
        self.physics_controller.vector_drive(xSpeed, ySpeed, rotation, tm_diff)
