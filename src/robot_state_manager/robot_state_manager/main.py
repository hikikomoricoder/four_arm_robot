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

    # ── Valid position values per group ───────────────────────────────
    _VALID_POSITIONS = {
        "veer":   frozenset({"home", "turn", "forward", "lift"}),
        "wheel":  frozenset({"home", "turn", "forward"}),
        "arm":    frozenset({"home", "low", "high", "rhombus_1", "rhombus_2",
                             "operate_1", "operate_2", "stabilize"}),
        "branch": frozenset({"home", "close"}),
    }

    # ── Valid status values per group ─────────────────────────────────
    _VALID_STATUSES = {
        "veer":   frozenset({"free", "occupy"}),
        "wheel":  frozenset({"free", "occupy"}),
        "arm":    frozenset({"free", "occupy"}),
        "branch": frozenset({"free", "occupy"}),
    }

    def __init__(self):
        super().__init__("state_manager")

        # ── Internal state store (all groups start at home/free) ──────
        self._states = {
            "veer":   {"position": "home", "status": "free"},
            "wheel":  {"position": "home", "status": "free"},
            "arm":    {"position": "home", "status": "free"},
            "branch": {"position": "home", "status": "free"},
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
        for group in ("veer", "wheel", "arm", "branch"):
            setattr(response, f"{group}_position", self._states[group]["position"])
            setattr(response, f"{group}_status",   self._states[group]["status"])

    # ── Get group ──────────────────────────────────────────────────────

    def _handle_get_group(self, request, response):
        group = request.group_name.strip().lower()
        if group not in self._states:
            raise ValueError(f"Unknown group '{request.group_name}'."
                             f" Valid: {', '.join(sorted(self._states))}")

        self._populate_all_states(response)
        s = self._states[group]
        response.success = True
        response.message = f"{group}: position={s['position']}, status={s['status']}"

    # ── Set group ──────────────────────────────────────────────────────

    def _handle_set_group(self, request, response):
        group = request.group_name.strip().lower()
        position = request.position_name.strip().lower()
        status = request.status_name.strip().lower()

        if group not in self._states:
            raise ValueError(f"Unknown group '{request.group_name}'."
                             f" Valid: {', '.join(sorted(self._states))}")

        changed = []

        if position:
            valid_pos = self._VALID_POSITIONS.get(group)
            if position not in valid_pos:
                raise ValueError(
                    f"Invalid position '{request.position_name}' for group"
                    f" '{group}'. Valid: {', '.join(sorted(valid_pos))}"
                )
            self._states[group]["position"] = position
            changed.append(f"position={position}")

        if status:
            valid_sta = self._VALID_STATUSES.get(group)
            if status not in valid_sta:
                raise ValueError(
                    f"Invalid status '{request.status_name}' for group"
                    f" '{group}'. Valid: {', '.join(sorted(valid_sta))}"
                )
            self._states[group]["status"] = status
            changed.append(f"status={status}")

        if not changed:
            raise ValueError(
                "At least one of position_name or status_name must be"
                " provided for set_group"
            )

        self.get_logger().info(f"State changed: {group} -> {', '.join(changed)}")

        self._populate_all_states(response)
        response.success = True
        response.message = f"Set {group}: {', '.join(changed)}"


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
