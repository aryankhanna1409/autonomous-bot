#!/usr/bin/env python3
"""Top-down 2D view of an rmf_demos world, every floor side by side.

Draws walls, doors, lanes, waypoints and lifts straight from the building.yaml,
then shows each robot live from /fleet_states and the last auction run by
auction_house.py (target star, winner line, bids).

    python3 swarm_view.py --map clinic
    python3 swarm_view.py --map clinic --snapshot clinic.png   # no ROS needed
"""
import argparse
import json
import math
import os
import tempfile
import time

import yaml
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

AUCTION_FILE = os.path.join(tempfile.gettempdir(), "swarm_auction.json")

FLEET_COLORS = {"deliveryRobot": "#d9480f", "tinyRobot": "#1971c2"}
GRAPH_COLORS = ["#ffa94d", "#74c0fc", "#b197fc"]


def find_building(map_name):
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("rmf_demos_maps")
    return os.path.join(share, map_name, f"{map_name}.building.yaml")


def load_building(path):
    with open(path) as f:
        building = yaml.safe_load(f)
    levels = {}
    for name, data in sorted(building["levels"].items(), key=lambda kv: kv[1].get("elevation", 0)):
        vertices = data["vertices"]
        a, b, params = data["measurements"][0]
        scale = params["distance"][1] / math.dist(vertices[a][:2], vertices[b][:2])  # metres per pixel
        levels[name] = {"data": data, "scale": scale}
    lifts = {name: (lift["x"], lift["y"]) for name, lift in building.get("lifts", {}).items()}
    return levels, lifts


def draw_level(ax, name, level, lifts):
    data = level["data"]
    V = data["vertices"]

    for floor in data.get("floors", []):
        ax.add_patch(Polygon([V[i][:2] for i in floor["vertices"]], closed=True,
                             facecolor="#f8f9fa", edgecolor="none", zorder=0))
    for lane in data.get("lanes", []):
        graph = lane[2].get("graph_idx", [2, 0])[1]
        (x1, y1), (x2, y2) = V[lane[0]][:2], V[lane[1]][:2]
        ax.plot([x1, x2], [y1, y2], color=GRAPH_COLORS[graph % len(GRAPH_COLORS)],
                linewidth=5, alpha=0.45, solid_capstyle="round", zorder=1)
    for wall in data.get("walls", []):
        (x1, y1), (x2, y2) = V[wall[0]][:2], V[wall[1]][:2]
        ax.plot([x1, x2], [y1, y2], color="#343a40", linewidth=1.6, zorder=2)
    for door in data.get("doors", []):
        (x1, y1), (x2, y2) = V[door[0]][:2], V[door[1]][:2]
        ax.plot([x1, x2], [y1, y2], color="#2f9e44", linewidth=3, zorder=2)

    for v in V:
        if not v[3]:
            continue
        params = v[4] if len(v) > 4 else {}
        if "is_charger" in params and params["is_charger"][1]:
            # Robots sit on their chargers, so the robot's own label covers it
            ax.plot(v[0], v[1], marker="s", markersize=5, color="#868e96", zorder=3)
            continue
        ax.plot(v[0], v[1], marker="o", markersize=4, color="#495057", zorder=3)
        label = v[3].replace(f"{name}_", "")
        ax.annotate(label, (v[0], v[1]), xytext=(4, 4), textcoords="offset points",
                    fontsize=6.5, color="#495057", zorder=3)

    for lift_name, (x, y) in lifts.items():
        ax.add_patch(plt.Rectangle((x - 35, y - 35), 70, 70, facecolor="#e5dbff",
                                   edgecolor="#7048e8", linewidth=1.2, zorder=2))
        ax.annotate(lift_name.replace("lift_", "lift\n"), (x, y), ha="center", va="center",
                    fontsize=5.5, color="#5f3dc4", zorder=3)

    ax.set_title(name, fontsize=13, fontweight="bold")
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def spawn_states(levels):
    """Robots at their spawn points, used until /fleet_states arrives."""
    robots = {}
    for level_name, level in levels.items():
        for v in level["data"]["vertices"]:
            params = v[4] if len(v) > 4 else {}
            if "spawn_robot_name" in params:
                model = params["spawn_robot_type"][1].split("/")[-1]
                robots[params["spawn_robot_name"][1]] = {
                    "fleet": model[0].lower() + model[1:], "level": level_name,
                    "px": v[0], "py": v[1], "yaw": None, "busy": False,
                }
    return robots


def read_auction():
    try:
        with open(AUCTION_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class SwarmView:
    def __init__(self, levels, lifts):
        self.levels = levels
        self.robots = spawn_states(levels)
        self.node = None
        self.ros_error = "not started"
        self.last_msg = None

        # Floors share one coordinate frame, so share axes to keep them lined up
        self.fig, axes = plt.subplots(1, len(levels), figsize=(7.5 * len(levels), 7),
                                      sharex=True, sharey=True, squeeze=False)
        axes = list(axes[0])
        axes[0].invert_yaxis()  # building.yaml y grows downwards like an image
        self.fig.canvas.manager.set_window_title("Swarm view")
        self.axes = dict(zip(levels, axes))
        for name, ax in self.axes.items():
            draw_level(ax, name, levels[name], lifts)
        self.status = self.fig.text(0.01, 0.01, "", fontsize=9, family="monospace", va="bottom")
        self.artists = []
        self.fig.tight_layout(rect=(0, 0.08, 1, 1))

    def attach_ros(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rmf_fleet_msgs.msg import FleetState

        rclpy.init()
        self.node = rclpy.create_node("swarm_view")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(FleetState, "/fleet_states", self.on_fleet_state, qos)
        self.ros_error = None

    def on_fleet_state(self, msg):
        if self.last_msg is None:
            print(f"Receiving /fleet_states (first from fleet '{msg.name}')")
        self.last_msg = time.monotonic()
        for robot in msg.robots:
            loc = robot.location
            level = self.levels.get(loc.level_name)
            if level is None:
                continue
            scale = level["scale"]
            self.robots[robot.name] = {
                "fleet": msg.name, "level": loc.level_name,
                "px": loc.x / scale, "py": -loc.y / scale, "yaw": loc.yaw,
                "busy": bool(robot.task_id),
            }

    def update(self, _frame=None):
        if self.node is not None:
            import rclpy
            rclpy.spin_once(self.node, timeout_sec=0.0)
            for _ in range(50):
                rclpy.spin_once(self.node, timeout_sec=0.0)

        for artist in self.artists:
            artist.remove()
        self.artists = []

        auction = read_auction()
        winner = auction["winner"] if auction else None

        for name, r in self.robots.items():
            ax = self.axes[r["level"]]
            color = FLEET_COLORS.get(r["fleet"], "#2b8a3e")
            ring = "#fab005" if name == winner else ("#212529" if r["busy"] else "white")
            self.artists.append(ax.scatter([r["px"]], [r["py"]], s=260, color=color,
                                           edgecolors=ring, linewidths=2.5, zorder=6))
            if r["yaw"] is not None:
                dx, dy = 32 * math.cos(r["yaw"]), -32 * math.sin(r["yaw"])
                self.artists += ax.plot([r["px"], r["px"] + dx], [r["py"], r["py"] + dy],
                                        color="white", linewidth=2, zorder=7)
            self.artists.append(ax.annotate(name, (r["px"], r["py"]), xytext=(9, -12),
                                            textcoords="offset points", fontsize=8,
                                            fontweight="bold", color=color, zorder=7))

        if auction:
            tx, ty, tlevel = auction["target_pos"]
            if tlevel in self.axes:
                ax = self.axes[tlevel]
                self.artists.append(ax.scatter([tx], [ty], marker="*", s=420, color="#fab005",
                                               edgecolors="#212529", linewidths=1, zorder=8))
                w = self.robots.get(winner)
                if w and w["level"] == tlevel:
                    self.artists += ax.plot([w["px"], tx], [w["py"], ty], color="#fab005",
                                            linestyle="--", linewidth=2, zorder=5)
            ranking = sorted(auction["bids"].items(), key=lambda kv: kv[1])
            bids = "  ".join(f"{r} {eta:.0f}s" for r, eta in ranking)
            self.status.set_text(f"Last auction: {auction['target']}  ->  WINNER {winner}\nBids: {bids}")
        else:
            self.status.set_text("No auction yet. Run auction_house.py in another terminal.")
        self.status.set_text(self.ros_line() + "\n" + self.status.get_text())
        return self.artists

    def ros_line(self):
        if self.ros_error:
            return f"ROS: OFF ({self.ros_error}), showing spawn positions only"
        if self.last_msg is None:
            return "ROS: waiting for /fleet_states (sim running? same ROS_DOMAIN_ID?)"
        age = time.monotonic() - self.last_msg
        return f"ROS: {'LIVE' if age < 2 else 'STALE'}, last /fleet_states {age:.1f}s ago"

    def run(self):
        self.anim = FuncAnimation(self.fig, self.update, interval=200, cache_frame_data=False)
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Top-down 2D swarm view of an rmf_demos world")
    parser.add_argument("--map", default="clinic", help="rmf_demos world name (clinic, office, ...)")
    parser.add_argument("--building", help="Path to the building.yaml (default: from the installed rmf_demos_maps)")
    parser.add_argument("--snapshot", help="Save one frame to this image file and exit (no ROS needed)")
    args = parser.parse_args()

    levels, lifts = load_building(args.building or find_building(args.map))
    view = SwarmView(levels, lifts)

    if args.snapshot:
        view.update()
        view.fig.savefig(args.snapshot, dpi=130)
        print(f"Saved {args.snapshot}")
        return

    try:
        view.attach_ros()
    except ImportError as e:
        view.ros_error = f"can't import {e.name}; source your ROS 2 + rmf workspace first"
        print(f"ROS 2 not available: {view.ros_error}")
    view.run()


if __name__ == "__main__":
    main()
