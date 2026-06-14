import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from sailbot import constants as c
from sailbot.utils.boatMath import distance_between
from sailbot.utils.utils import Waypoint
from sailbot_interfaces.msg import Waypoint as WaypointMsg
from sailbot_interfaces.msg import WaypointQueueState
from sailbot_interfaces.srv import (
    AddWaypoint,
    ClearWaypointQueue,
    DeleteWaypoint,
    GetWaypointQueue,
    ReorderWaypoints,
    SetCurrentIndex,
    SetWaypointQueue,
    UpdateWaypoint,
)


@dataclass(slots=True)
class ManagedWaypoint:
    lat: float
    lon: float
    name: str = ""

    def to_msg(self) -> WaypointMsg:
        msg = WaypointMsg()
        msg.lat = float(self.lat)
        msg.lon = float(self.lon)
        msg.name = self.name
        return msg

    @staticmethod
    def from_msg(msg: WaypointMsg) -> "ManagedWaypoint":
        return ManagedWaypoint(lat=float(msg.lat), lon=float(msg.lon), name=msg.name)


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")
        self.logging = self.get_logger()

        self.waypoint_tolerance = float(c.config["CONSTANTS"]["reached_waypoint_distance"])

        base_path = os.environ.get("WAYPOINT_QUEUE_FILE")
        if base_path:
            self.storage_path = Path(base_path)
        elif Path("/workspace").exists():
            self.storage_path = Path("/workspace/data/waypoint_queue_state.json")
        else:
            self.storage_path = Path.cwd() / "data" / "waypoint_queue_state.json"

        self.waypoints: list[ManagedWaypoint] = []
        self.current_index = 0
        self.version = 0
        self.boat_position: Waypoint | None = None

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_pub = self.create_publisher(WaypointQueueState, "/waypoint_queue_state", state_qos)
        self.next_gps_pub = self.create_publisher(String, "/next_gps", 10)

        self.legacy_next_gps_sub = self.create_subscription(String, "/next_gps_command", self._legacy_append_callback, 10)
        self.gps_sub = self.create_subscription(String, "/GPS", self._gps_callback, 10)

        self.create_service(AddWaypoint, "/waypoint_manager/add_waypoint", self._add_waypoint)
        self.create_service(UpdateWaypoint, "/waypoint_manager/update_waypoint", self._update_waypoint)
        self.create_service(DeleteWaypoint, "/waypoint_manager/delete_waypoint", self._delete_waypoint)
        self.create_service(ReorderWaypoints, "/waypoint_manager/reorder_waypoints", self._reorder_waypoints)
        self.create_service(SetWaypointQueue, "/waypoint_manager/set_waypoint_queue", self._set_waypoint_queue)
        self.create_service(SetCurrentIndex, "/waypoint_manager/set_current_index", self._set_current_index)
        self.create_service(ClearWaypointQueue, "/waypoint_manager/clear", self._clear_waypoint_queue)
        self.create_service(GetWaypointQueue, "/waypoint_manager/get", self._get_waypoint_queue)

        self._load_state()
        self._publish_state()
        self.logging.info("WaypointManager started")

    def _get_state_msg(self) -> WaypointQueueState:
        state = WaypointQueueState()
        state.stamp = self.get_clock().now().to_msg()
        state.version = int(self.version)
        state.current_index = int(self.current_index)
        state.waypoints = [wp.to_msg() for wp in self.waypoints]
        return state

    def _serialize(self) -> dict:
        return {
            "version": int(self.version),
            "current_index": int(self.current_index),
            "waypoints": [asdict(wp) for wp in self.waypoints],
        }

    def _persist_state(self):
        self.storage_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.storage_path.with_suffix(self.storage_path.suffix + ".tmp")
        with tmp_path.open("w", encoding="utf-8") as f:
            json.dump(self._serialize(), f)
        tmp_path.replace(self.storage_path)

    def _load_state(self):
        if not self.storage_path.exists():
            return

        try:
            with self.storage_path.open("r", encoding="utf-8") as f:
                payload = json.load(f)

            self.version = int(payload.get("version", 0))
            self.current_index = int(payload.get("current_index", 0))
            self.waypoints = []
            for item in payload.get("waypoints", []):
                self.waypoints.append(
                    ManagedWaypoint(
                        lat=float(item["lat"]),
                        lon=float(item["lon"]),
                        name=str(item.get("name", "")),
                    )
                )

            if len(self.waypoints) == 0:
                self.current_index = 0
            elif self.current_index >= len(self.waypoints):
                self.current_index = len(self.waypoints) - 1

            self.logging.info(f"Loaded {len(self.waypoints)} persisted waypoints")
        except Exception as exc:
            self.logging.error(f"Failed to load waypoint state: {exc}")

    def _publish_target(self):
        if len(self.waypoints) == 0 or self.current_index >= len(self.waypoints):
            self.next_gps_pub.publish(String(data=json.dumps({"lat": None, "lon": None})))
            return

        target = self.waypoints[self.current_index]
        msg = Waypoint(target.lat, target.lon).to_msg()
        self.next_gps_pub.publish(msg)

    def _publish_state(self):
        self.state_pub.publish(self._get_state_msg())
        self._publish_target()

    def _commit_state(self, reason: str):
        self.version += 1
        self._persist_state()
        self._publish_state()
        self.logging.info(f"Waypoint queue updated ({reason}): {len(self.waypoints)} waypoint(s), current index {self.current_index}")

    def _normalize_index(self):
        if len(self.waypoints) == 0:
            self.current_index = 0
            return
        self.current_index = max(0, min(self.current_index, len(self.waypoints) - 1))

    def _legacy_append_callback(self, msg: String):
        wp = Waypoint.from_msg(msg)
        if wp is None:
            return

        if any(existing.lat == wp.lat and existing.lon == wp.lon for existing in self.waypoints):
            return

        self.waypoints.append(ManagedWaypoint(lat=wp.lat, lon=wp.lon, name=""))
        self._normalize_index()
        self._commit_state("legacy append")

    def _gps_callback(self, msg: String):
        self.boat_position = Waypoint.from_msg(msg)
        if self.boat_position is None:
            return
        if len(self.waypoints) == 0 or self.current_index >= len(self.waypoints):
            return

        current = self.waypoints[self.current_index]
        target = Waypoint(current.lat, current.lon)
        if distance_between(self.boat_position, target) < self.waypoint_tolerance:
            self.current_index += 1
            if self.current_index > len(self.waypoints):
                self.current_index = len(self.waypoints)
            self._commit_state("auto advance")

    def _add_waypoint(self, request: AddWaypoint.Request, response: AddWaypoint.Response):
        wp = ManagedWaypoint.from_msg(request.waypoint)
        idx = int(request.index)

        if idx < 0 or idx >= len(self.waypoints):
            self.waypoints.append(wp)
        else:
            self.waypoints.insert(idx, wp)
            if idx <= self.current_index and len(self.waypoints) > 1:
                self.current_index += 1

        self._normalize_index()
        self._commit_state("add waypoint")

        response.success = True
        response.message = "Waypoint added"
        response.state = self._get_state_msg()
        return response

    def _update_waypoint(self, request: UpdateWaypoint.Request, response: UpdateWaypoint.Response):
        idx = int(request.index)
        if idx < 0 or idx >= len(self.waypoints):
            response.success = False
            response.message = "Index out of range"
            response.state = self._get_state_msg()
            return response

        self.waypoints[idx] = ManagedWaypoint.from_msg(request.waypoint)
        self._commit_state("update waypoint")

        response.success = True
        response.message = "Waypoint updated"
        response.state = self._get_state_msg()
        return response

    def _delete_waypoint(self, request: DeleteWaypoint.Request, response: DeleteWaypoint.Response):
        idx = int(request.index)
        if idx < 0 or idx >= len(self.waypoints):
            response.success = False
            response.message = "Index out of range"
            response.state = self._get_state_msg()
            return response

        self.waypoints.pop(idx)
        if idx < self.current_index:
            self.current_index -= 1
        self._normalize_index()
        self._commit_state("delete waypoint")

        response.success = True
        response.message = "Waypoint deleted"
        response.state = self._get_state_msg()
        return response

    def _reorder_waypoints(self, request: ReorderWaypoints.Request, response: ReorderWaypoints.Response):
        order = [int(i) for i in request.order]
        if len(order) != len(self.waypoints):
            response.success = False
            response.message = "Order length must match waypoint count"
            response.state = self._get_state_msg()
            return response

        expected = list(range(len(self.waypoints)))
        if sorted(order) != expected:
            response.success = False
            response.message = "Order must be a permutation of current indices"
            response.state = self._get_state_msg()
            return response

        old_waypoints = self.waypoints
        old_current = self.current_index
        inverse = {old_idx: new_idx for new_idx, old_idx in enumerate(order)}
        self.waypoints = [old_waypoints[old_idx] for old_idx in order]
        if len(self.waypoints) > 0 and old_current < len(self.waypoints):
            self.current_index = inverse.get(old_current, 0)
        else:
            self.current_index = 0

        self._normalize_index()
        self._commit_state("reorder waypoints")

        response.success = True
        response.message = "Waypoints reordered"
        response.state = self._get_state_msg()
        return response

    def _set_waypoint_queue(self, request: SetWaypointQueue.Request, response: SetWaypointQueue.Response):
        self.waypoints = [ManagedWaypoint.from_msg(wp) for wp in request.waypoints]
        self.current_index = int(request.current_index)
        self._normalize_index()
        self._commit_state("set waypoint queue")

        response.success = True
        response.message = "Waypoint queue replaced"
        response.state = self._get_state_msg()
        return response

    def _set_current_index(self, request: SetCurrentIndex.Request, response: SetCurrentIndex.Response):
        if len(self.waypoints) == 0:
            self.current_index = 0
            self._commit_state("set current index on empty queue")
            response.success = True
            response.message = "Queue is empty"
            response.state = self._get_state_msg()
            return response

        idx = int(request.current_index)
        if idx < 0 or idx >= len(self.waypoints):
            response.success = False
            response.message = "Index out of range"
            response.state = self._get_state_msg()
            return response

        self.current_index = idx
        self._commit_state("set current index")

        response.success = True
        response.message = "Current index updated"
        response.state = self._get_state_msg()
        return response

    def _clear_waypoint_queue(self, request: ClearWaypointQueue.Request, response: ClearWaypointQueue.Response):
        self.waypoints = []
        self.current_index = 0
        self._commit_state("clear queue")

        response.success = True
        response.message = "Waypoint queue cleared"
        response.state = self._get_state_msg()
        return response

    def _get_waypoint_queue(self, request: GetWaypointQueue.Request, response: GetWaypointQueue.Response):
        response.success = True
        response.message = "OK"
        response.state = self._get_state_msg()
        return response


def main(args=None):
    os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/waypoint_manager"
    rclpy.init(args=args)

    node = WaypointManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
