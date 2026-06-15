#!/usr/bin/env python3
import unittest

import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.srv import GetParameters, SetParameters
from cartesian_compliance_controller.srv import (  # type: ignore[attr-defined]
    SetStiffness,
)
from typing import Any


def generate_test_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("cartesian_controller_simulation"),
                "launch",
                "simulation.launch.py",
            ]
        )
    )
    until_ready = 10.0  # sec
    return LaunchDescription(
        [
            setup,
            TimerAction(
                period=until_ready, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )


class IntegrationTest(unittest.TestCase):
    """Integration tests for the cartesian controllers in a simulation setting

    We check if each controller successfully performs the life cycle of
    `initialized` - `active` - `update` - `inactive`, and whether it behaves
    correctly in selected use cases.
    """

    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_startup")

        cls.our_controllers = [
            "cartesian_motion_controller",
            "cartesian_force_controller",
            "cartesian_compliance_controller",
            "motion_control_handle",
        ]
        cls.invalid_controllers = [
            "invalid_cartesian_force_controller",
            "invalid_cartesian_compliance_controller",
        ]

        cls.setup_interfaces(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setup_interfaces(self):
        """Setup interfaces for ROS2 services and topics"""

        timeout = rclpy.time.Duration(seconds=5)
        self.list_controllers = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        if not self.list_controllers.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service list_controllers not available")

        self.switch_controller = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        if not self.switch_controller.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service switch_controllers not available")

        self.get_parameter_clients = {
            controller: self.node.create_client(
                GetParameters, f"/{controller}/get_parameters"
            )
            for controller in self.our_controllers[0:3]
        }

        self.set_parameter_clients = {
            controller: self.node.create_client(
                SetParameters, f"/{controller}/set_parameters"
            )
            for controller in self.our_controllers[0:3]
        }

        self.target_pose_pub = self.node.create_publisher(
            PoseStamped, "target_frame", 3
        )
        self.target_wrench_pub = self.node.create_publisher(
            WrenchStamped, "target_wrench", 3
        )
        self.ft_sensor_wrench_pub = self.node.create_publisher(
            WrenchStamped, "ft_sensor_wrench", 3
        )

    def test_controller_initialization(self):
        """Test whether every controller got initialized correctly

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have `state:
        inactive`.
        """
        for name in self.our_controllers:
            self.assertTrue(
                self.check_state(name, "inactive"), f"{name} is initialized correctly"
            )

    def test_invalid_controller_initialization(self):
        """Test whether the invalid controllers' initialization fails as expected

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have the
        expected state.
        """
        expected_state = "unconfigured"
        for name in self.invalid_controllers:
            self.assertTrue(
                self.check_state(name, expected_state),
                f"{name} initializes although it should not.",
            )

    def test_controller_switches(self):
        """Test whether every controller starts, runs, and stops correctly

        We start each of our controllers individually and check if its state is
        `active`.  We then switch it off and check whether its state is
        `inactive`.
        """
        for name in self.our_controllers:
            self.start_controller(name)
            self.assertTrue(
                self.check_state(name, "active"),
                "{} is starting correctly".format(name),
            )
            time.sleep(3)  # process some update() cycles
            self.stop_controller(name)
            self.assertTrue(
                self.check_state(name, "inactive"),
                "{} is stopping correctly".format(name),
            )

    def test_inputs_with_nans(self):
        """Test whether every controller survives inputs with NaNs

        We publish invalid values to all input topics.
        The controllers' callbacks store them even when in inactive state after startup.
        We then check if every controller can be activated normally.
        """

        def publish_nan_targets():
            # Target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position.x = float("nan")
            target_pose.pose.position.y = float("nan")
            target_pose.pose.position.z = float("nan")
            target_pose.pose.orientation.x = float("nan")
            target_pose.pose.orientation.y = float("nan")
            target_pose.pose.orientation.z = float("nan")
            target_pose.pose.orientation.w = float("nan")
            self.target_pose_pub.publish(target_pose)

            # Force-torque sensor
            ft_sensor_wrench = WrenchStamped()
            ft_sensor_wrench.wrench.force.x = float("nan")
            ft_sensor_wrench.wrench.force.y = float("nan")
            ft_sensor_wrench.wrench.force.z = float("nan")
            ft_sensor_wrench.wrench.torque.x = float("nan")
            ft_sensor_wrench.wrench.torque.y = float("nan")
            ft_sensor_wrench.wrench.torque.z = float("nan")
            self.ft_sensor_wrench_pub.publish(ft_sensor_wrench)

            # Target wrench
            target_wrench = WrenchStamped()
            target_wrench.wrench.force.x = float("nan")
            target_wrench.wrench.force.y = float("nan")
            target_wrench.wrench.force.z = float("nan")
            target_wrench.wrench.torque.x = float("nan")
            target_wrench.wrench.torque.y = float("nan")
            target_wrench.wrench.torque.z = float("nan")
            self.target_wrench_pub.publish(target_wrench)

        for name in self.our_controllers:
            self.start_controller(name)
            publish_nan_targets()
            time.sleep(3)  # process some update() cycles
            self.assertTrue(
                self.check_state(name, "active"), f"{name} survives inputs with NaNs"
            )
            self.stop_controller(name)

    def test_solver_parameters(self):
        """Check whether we can set and get nested solver parameters"""
        example_param = "solver.forward_dynamics.link_mass"
        default_value = 0.1
        new_value = 0.7

        for client in self.get_parameter_clients.values():
            result = self.get_parameters(client, [example_param])
            result = result.values[0].double_value
            self.assertTrue(result == default_value)

        for client in self.set_parameter_clients.values():
            param = Parameter(
                name=example_param,
                value=ParameterValue(
                    double_value=new_value, type=ParameterType.PARAMETER_DOUBLE
                ),
            )
            self.set_parameters(client, [param])

        for client in self.get_parameter_clients.values():
            result = self.get_parameters(client, [example_param])
            result = result.values[0].double_value
            self.assertTrue(result == new_value)

    def test_compliance_parallel_stiffness_parameters(self):
        """Check parallel force-position and stiffness selection parameters"""
        client = self.set_parameter_clients["cartesian_compliance_controller"]
        params = [
            Parameter(
                name="stiffness.use_parallel_force_position_control",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=True
                ),
            ),
            Parameter(
                name="stiffness.sel_z",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=0.0
                ),
            ),
            Parameter(
                name="stiffness.trans_z",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=250.0
                ),
            ),
        ]
        self.set_parameters(client, params)

        result = self.get_parameters(
            self.get_parameter_clients["cartesian_compliance_controller"],
            [
                "stiffness.use_parallel_force_position_control",
                "stiffness.sel_z",
                "stiffness.trans_z",
            ],
        )
        self.assertTrue(result.values[0].bool_value)
        self.assertAlmostEqual(result.values[1].double_value, 0.0)
        self.assertAlmostEqual(result.values[2].double_value, 250.0)

    def test_end_effector_link_update_while_inactive(self):
        """Runtime end_effector_link changes are accepted while inactive"""
        controller = "cartesian_motion_controller"
        get_client = self.get_parameter_clients[controller]
        set_client = self.set_parameter_clients[controller]

        result = self.get_parameters(get_client, ["end_effector_link"])
        self.assertEqual(result.values[0].string_value, "tool0")

        param = Parameter(
            name="end_effector_link",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="tool0"
            ),
        )
        self.set_parameters(set_client, [param])

        result = self.get_parameters(get_client, ["end_effector_link"])
        self.assertEqual(result.values[0].string_value, "tool0")

    def test_wrench_state_feedback(self):
        """Wrench feedback topics publish when state feedback is enabled"""
        received = {"target": False, "net": False}

        def target_cb(_msg):
            received["target"] = True

        def net_cb(_msg):
            received["net"] = True

        self.node.create_subscription(
            WrenchStamped,
            "/cartesian_force_controller/current_target_wrench",
            target_cb,
            10,
        )
        self.node.create_subscription(
            WrenchStamped,
            "/cartesian_force_controller/current_net_force_wrench",
            net_cb,
            10,
        )

        self.start_controller("cartesian_force_controller")

        target_wrench = WrenchStamped()
        target_wrench.header.frame_id = "base_link"
        target_wrench.wrench.force.z = 5.0
        for _ in range(20):
            self.target_wrench_pub.publish(target_wrench)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(received["target"])
        self.assertTrue(received["net"])
        self.stop_controller("cartesian_force_controller")

    def test_set_stiffness_service(self):
        """SetStiffness service accepts a full 6x6 matrix"""
        client = self.node.create_client(
            SetStiffness, "/cartesian_compliance_controller/set_stiffness"
        )
        timeout = rclpy.time.Duration(seconds=5)
        self.assertTrue(client.wait_for_service(timeout.nanoseconds / 1e9))

        request = SetStiffness.Request()
        request.stiffness = [0.0] * 36
        for i in range(6):
            request.stiffness[i * 6 + i] = 100.0 + i

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        response = future.result()
        self.assertTrue(response.success)

    def check_state(self, controller, state):
        """Check the controller's state

        Return True if the controller's state is `state`, else False.
        Return False if the controller is not listed.
        """
        req = ListControllers.Request()
        future = self.list_controllers.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        for entry in future.result().controller:
            if entry.name == controller:
                return True if entry.state == state else False
        return False

    def start_controller(self, controller):
        """Start the given controller"""
        req = SwitchController.Request()
        req.activate_controllers = [controller]
        self.perform_switch(req)

    def stop_controller(self, controller):
        """Stop the given controller"""
        req = SwitchController.Request()
        req.deactivate_controllers = [controller]
        self.perform_switch(req)

    def perform_switch(self, req):
        """Trigger the controller switch with the given request"""
        req.strictness = req.BEST_EFFORT
        future = self.switch_controller.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def set_parameters(self, client: Client, params: list[Parameter]) -> None:
        req = SetParameters.Request()
        req.parameters = params
        future = client.call_async(req)
        rclpy.spin_until_future_complete(
            self.node, future  # type: ignore[attr-defined]
        )

    def get_parameters(self, client: Client, names: list[str]) -> Any:
        req = GetParameters.Request()
        req.names = names
        future = client.call_async(req)
        rclpy.spin_until_future_complete(
            self.node, future  # type: ignore[attr-defined]
        )
        return future.result()
