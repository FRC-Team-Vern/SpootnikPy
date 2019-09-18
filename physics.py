import pprint
import logging

from pyfrc.physics import drivetrains
from pyfrc.sim import get_user_renderer

from Box2D import b2World, b2PolygonShape


class PhysicsEngine:

    FT_TO_METER_CONV = 0.3048

    def __init__(self, physics_controller):
        self.logger = logging.getLogger("PhysicsEngine")
        self.physics_controller = physics_controller
        self.sim_type = self.physics_controller.config_obj["pyfrc"]["sim_type"]
        self.logger.info(f"Simulation type: {self.sim_type}")
        self.drivetrain = drivetrains.MecanumDrivetrain()
        self.initial = True
        self.simulated_position = 0.0
        self.user_renderer = get_user_renderer()

        # Box2d-related
        self.world = None
        self.field_height = None
        self.ground = None
        self.robot_body = None
        self.rear_wheel = None
        self.front_wheel = None
        self.vel_iters = 6
        self.pos_iters = 2

    def initialize(self, hal_data):
        if self.sim_type == "profile":
            self.field_height = self.physics_controller.config_obj["pyfrc"]["profile"]["field"]["h"]

            self.world = b2World()  # default gravity is (0,-10) and doSleep is True
            try:
                config_field_objects = self.physics_controller.config_obj["pyfrc"]["profile"]["field"]["objects"]
                config_robot_objects = self.physics_controller.config_obj["pyfrc"]["profile"]["robot"]["objects"]
            except KeyError:
                config_field_objects = []

            ground_config_object = next((obj for obj in config_field_objects if "name" in obj and obj["name"] == "ground"),
                                        None)
            if ground_config_object:
                ground_center_x, ground_center_y, ground_width, ground_height = \
                    self.convertConfigToBox2DBox(ground_config_object)
            else:
                ground_center_x, ground_center_y, ground_width, ground_height = 0, -10, 50, 10

            self.ground = self.world.CreateStaticBody(position=(ground_center_x, ground_center_y),
                                                      shapes=b2PolygonShape(box=(ground_width, ground_height)))
            self.robot_body = self.world.CreateDynamicBody(position=(0, 4))

            # Add a box fixture to the robot body
            self.robot_body.CreatePolygonFixture(box=(1, 1), density=1, friction=0.3)

            rear_wheel_object = next((obj for obj in config_robot_objects if "name" in obj and obj["name"] == "rear_wheel"),
                                     None)
            front_wheel_object = next((obj for obj in config_robot_objects if "name" in obj and obj["name"] == "front_wheel"),
                                      None)

            # Only create wheels if both objects exist
            if rear_wheel_object and front_wheel_object:
                rear_wheel_center = self.convertConfigToBox2DCoords(rear_wheel_object["center"])
                self.rear_wheel = self.world.CreateDynamicBody(position=rear_wheel_center)
                rear_wheel_points = [self.convertConfigToBox2DCoords(pt) for pt in rear_wheel_object["points"]]
                self.rear_wheel.CreatePolygonFixture(vertices=rear_wheel_points)
                front_wheel_center = self.convertConfigToBox2DCoords(front_wheel_object["center"])
                self.front_wheel = self.world.CreateDynamicBody(position=front_wheel_center)
                front_wheel_points = [self.convertConfigToBox2DCoords(pt) for pt in front_wheel_object["points"]]
                self.front_wheel.CreatePolygonFixture(vertices=front_wheel_points)

    def update_sim(self, hal_data, now, tm_diff):
        # print(hal_data)
        if self.initial:
            self.initial = False
            # pprint.pprint(hal_data["CAN"]["sparkmax-5"])
            pprint.pprint(hal_data)
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

        if self.sim_type == "top":
            xSpeed, ySpeed, rotation = self.drivetrain.get_vector(lr_motor, rr_motor, lf_motor, rf_motor)
            self.physics_controller.vector_drive(xSpeed, ySpeed, rotation, tm_diff)

        elif self.sim_type == "profile":
            print(f"User Program State: {hal_data['user_program_state']}")
            if hal_data['user_program_state'] == "teleop":
                self.world.Step(tm_diff, self.vel_iters, self.pos_iters)

                # Clear applied body forced. We didn't apply any forced, but you should know about this function.
                self.world.ClearForces()

                self.logger.info(f"Position: {self.robot_body.position}, Angle: {self.robot_body.angle}")
                self.logger.info(f"Rear Position: {self.rear_wheel.position}")
                self.logger.info(f"Front Position: {self.front_wheel.position}")

                rear_wheel_x, rear_wheel_y = self.convertConfigToBox2DCoords((self.rear_wheel.position.x,
                                                                              self.rear_wheel.position.y))
                front_wheel_x, front_wheel_y = self.convertConfigToBox2DCoords((self.front_wheel.position.x,
                                                                                self.front_wheel.position.y))
                self.physics_controller.update_element_position("rear_wheel", rear_wheel_x, rear_wheel_y, 0.0)
                self.physics_controller.update_element_position("front_wheel", front_wheel_x, front_wheel_y, 0.0)

    def convertConfigToBox2DBox(self, config_box_object):
        assert "points" in config_box_object and len(config_box_object["points"]) == 4, \
            "Config box objects must have 4 points"

        config_box_pts = config_box_object["points"]
        ul = self.convertConfigToBox2DCoords(config_box_pts[0])
        ur = self.convertConfigToBox2DCoords(config_box_pts[1])
        lr = self.convertConfigToBox2DCoords(config_box_pts[2])
        ll = self.convertConfigToBox2DCoords(config_box_pts[3])

        center_x = (ul[0] + ur[0]) * 0.5
        center_y = (ul[1] + ll[1]) * 0.5
        box2d_width = (ur[0] - ul[0]) * 0.5
        box2d_height = (ul[1] - ll[1]) * 0.5

        return center_x, center_y, box2d_width, box2d_height

    def convertConfigToBox2DCoords(self, pt):
        return pt[0], self.field_height - pt[1]
