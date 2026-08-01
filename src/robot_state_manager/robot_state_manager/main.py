#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_interfaces.srv import GroupStateManager


class StateManagerNode(Node):
    """ROS2 service node for robot joint group state management.

    Provides a single ``/group_state_manager`` service supporting:
      - ``get_all``   — return states of all groups
      - ``get_group`` — return state of a named group
      - ``set_group`` — update the state of a named group (with validation)
    """

    # ── Valid state values per group ──────────────────────────────────
    _VALID_STATES = {
        "veer":   frozenset({"init", "turn", "forward", "free", "occupy"}),
        "wheel":  frozenset({"init", "turn", "forward", "free", "occupy"}),
        "arm":    frozenset({"init", "low", "narrow",
                             "operate_1", "operate_2", "stabilize",
                             "free", "occupy"}),
        "branch": frozenset({"init", "close", "free", "occupy"}),
    }

    def __init__(self):
        super().__init__("state_manager")

        # ── Internal state store (all groups start at "init") ────────
        self._states = {
            "veer":   "init",
            "wheel":  "init",
            "arm":    "init",
            "branch": "init",
        }

        # ── Service ───────────────────────────────────────────────────
        self._srv = self.create_service(
            GroupStateManager, "group_state_manager", self._handle_request
        )
        self.get_logger().info("StateManager node ready on /group_state_manager")

    # ── Request handler ────────────────────────────────────────────────

    def _handle_request(self, request, response):
        cmd = request.command.strip().lower()

        try:
            if cmd == "get_all":
                self._populate_all_states(response)
                response.success = True
                response.message = "OK"
            elif cmd == "get_group":
                self._handle_get_group(request, response)
            elif cmd == "set_group":
                self._handle_set_group(request, response)
            else:
                raise ValueError(f"Unknown command '{request.command}'."
                                 f" Expected: get_all, get_group, set_group")
        except (ValueError, KeyError) as e:
            response.success = False
            response.message = str(e)

        return response

    # ── Get all ────────────────────────────────────────────────────────

    def _populate_all_states(self, response):
        response.veer_state = self._states["veer"]
        response.wheel_state = self._states["wheel"]
        response.arm_state = self._states["arm"]
        response.branch_state = self._states["branch"]

    # ── Get group ──────────────────────────────────────────────────────

    def _handle_get_group(self, request, response):
        group = request.group_name.strip().lower()
        if group not in self._states:
            raise ValueError(f"Unknown group '{request.group_name}'."
                             f" Valid: {', '.join(sorted(self._states))}")

        self._populate_all_states(response)
        response.success = True
        response.message = f"{group} = {self._states[group]}"

    # ── Set group ──────────────────────────────────────────────────────

    def _handle_set_group(self, request, response):
        group = request.group_name.strip().lower()
        state = request.state_name.strip().lower()

        if group not in self._states:
            raise ValueError(f"Unknown group '{request.group_name}'."
                             f" Valid: {', '.join(sorted(self._states))}")

        valid = self._VALID_STATES.get(group)
        if state not in valid:
            raise ValueError(
                f"Invalid state '{request.state_name}' for group '{group}'."
                f" Valid: {', '.join(sorted(valid))}"
            )

        self._states[group] = state
        self.get_logger().info(f"State changed: {group} -> {state}")

        self._populate_all_states(response)
        response.success = True
        response.message = f"Set {group} = {state}"


def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
