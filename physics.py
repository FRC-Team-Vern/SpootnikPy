import pprint
import logging

from pyfrc.physics import drivetrains
from pyfrc.sim import get_user_renderer


class PhysicsEngine:

    def __init__(self, physics_controller):
        self.logger = logging.getLogger("PhysicsEngine")
        self.physics_controller = physics_controller
        self.sim_type = self.physics_controller.config_obj["pyfrc"]["sim_type"]
        self.logger.info(f"Simulation type: {self.sim_type}")
        self.drivetrain = drivetrains.MecanumDrivetrain()
        self.initial = True
        self.simulated_position = 0.0
        self.user_renderer = get_user_renderer()
        # self.global_field = self.user_renderer._global_ui.field

    def update_sim(self, hal_data, now, tm_diff):
        # print(hal_data)
        # if self.initial:
        #     self.initial = False
        #     pprint.pprint(hal_data["CAN"]["sparkmax-5"])
            # for key in hal_data["sparkmax-5"].keys():
            #     print(key)
        # Simulate the drivetrain
        lf_motor = hal_data["CAN"][4]["value"]
        lr_motor = hal_data["CAN"][1]["value"]
        rf_motor = -hal_data["CAN"][2]["value"]
        rr_motor = -hal_data["CAN"][3]["value"]

        # Control the flipper sparkmax-10 position
        if False:
            current_position = hal_data["CAN"]["sparkmax-10"]["position"]
            value = hal_data["CAN"]["sparkmax-10"]["value"]

            self.logger.info("Current position: {}".format(current_position))
            self.logger.info("sparkmax-10 value: {}".format(value))

            if current_position < 0:
                gravity_factor = 0.25
                hal_data["CAN"]["sparkmax-10"]["position"] = current_position + value + gravity_factor
            else:
                hal_data["CAN"]["sparkmax-10"]["position"] = current_position + value

        # Control the CargoEffector
        if False:
            cargo_effector_motor = hal_data["CAN"][6]["value"]
            self.logger.info("CargoEffector value: {}".format(cargo_effector_motor))

        xSpeed, ySpeed, rotation = self.drivetrain.get_vector(lr_motor, rr_motor, lf_motor, rf_motor)
        self.physics_controller.vector_drive(xSpeed, ySpeed, rotation, tm_diff)
        self.physics_controller.update_element_position("left_wheel", 0.5, 0.5, 0.0)
        self.physics_controller.update_element_position("right_wheel", 0.5, -0.5, 0.0)
