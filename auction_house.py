#!/usr/bin/env python3
import argparse
import json
import math
import os
import subprocess
import tempfile
import time

# Positions are (x, y, level) in building.yaml pixel coordinates.
# Each robot bids only on waypoints its fleet's nav graph can reach.
MAPS = {
    "office": {
        "locations": {
            'pantry': (1990.00, 638.36, 'L1'),
            'lounge': (2438.39, 471.24, 'L1'),
            'supplies': (770.88, 383.43, 'L1'),
            'hardware_2': (2474.55, 885.64, 'L1'),
            'coe': (631.56, 587.89, 'L1'),
            'patrol_A1': (1191.44, 824.46, 'L1'),
            'patrol_A2': (2238.47, 477.12, 'L1'),
            'patrol_B': (943.63, 1273.43, 'L1'),
            'patrol_D1': (1991.12, 812.87, 'L1'),
            'patrol_D2': (1210.54, 365.25, 'L1')
        },
        "fleets": {
            # speed is in map pixels per second
            "tinyRobot": {"speed": 30.0, "reach": None},
        },
        "robots": {
            "tinyRobot1": {"fleet": "tinyRobot", "pos": (1232.42, 658.57, 'L1')},
            "tinyRobot2": {"fleet": "tinyRobot", "pos": (2412.58, 627.50, 'L1')},
            "tinyRobot3": {"fleet": "tinyRobot", "pos": (1365.11, 1120.36, 'L1')}
        },
        "lifts": [],
    },
    "clinic": {
        # Robot chargers (L1_sub_waiting_area_1/5, L2_sub_waiting_area_4) are
        # left out so the auction never sends a robot onto a parked one.
        "locations": {
            'L1_left_treatment_1': (375.13, 505.42, 'L1'),
            'L1_left_negative_air_pressure': (379.74, 997.57, 'L1'),
            'L1_sub_waiting_area_2': (566.19, 382.07, 'L1'),
            'L1_left_nurse_center': (799.49, 728.99, 'L1'),
            'L1_sub_waiting_area_3': (1369.50, 372.68, 'L1'),
            'L1_right_nurse_center': (1519.39, 750.88, 'L1'),
            'L1_right_procedure': (1943.08, 499.60, 'L1'),
            'L1_right_treatment': (1914.74, 735.04, 'L1'),
            'L1_right_negative_air_pressure': (1896.75, 1153.90, 'L1'),
            'L1_sub_waiting_sucks_6': (1148.81, 1249.47, 'L1'),
            'L1_sub_waiting_area_4': (1151.09, 642.95, 'L1'),
            'L1_sub_waiting_area_7': (1890.78, 1284.05, 'L1'),
            'L1_sub_waiting_area_8': (1849.82, 1619.12, 'L1'),
            'L2_north_counter': (1117.45, 302.50, 'L2'),
            'L2_west_counter': (307.13, 650.84, 'L2'),
            'L2_east_counter': (2030.23, 1054.39, 'L2'),
            'L2_south_counter': (1304.19, 1511.26, 'L2'),
            'L2_left_nurse_center': (799.16, 735.98, 'L2'),
            'L2_right_nurse_center': (1507.68, 756.65, 'L2'),
            'L2_sub_waiting_area_1': (301.74, 1249.74, 'L2'),
            'L2_sub_waiting_area_2': (1151.38, 640.93, 'L2'),
            'L2_sub_waiting_area_3': (1152.91, 1019.84, 'L2')
        },
        "fleets": {
            # Clinic map scale is ~0.0269 m/px; speeds come from the fleet configs
            "deliveryRobot": {
                "speed": 0.7 / 0.0269,
                "reach": None,  # nav graph 0 covers every location above
            },
            "tinyRobot": {
                "speed": 0.5 / 0.0269,
                "reach": {  # nav graph 1 only covers part of L1
                    'L1_left_nurse_center', 'L1_right_nurse_center',
                    'L1_sub_waiting_area_3', 'L1_sub_waiting_area_4',
                    'L2_north_counter', 'L2_west_counter', 'L2_east_counter',
                    'L2_south_counter', 'L2_left_nurse_center',
                    'L2_right_nurse_center', 'L2_sub_waiting_area_1',
                    'L2_sub_waiting_area_2', 'L2_sub_waiting_area_3'
                },
            },
        },
        "robots": {
            "deliveryRobot_1": {"fleet": "deliveryRobot", "pos": (297.42, 1178.75, 'L1')},
            "deliveryRobot_2": {"fleet": "deliveryRobot", "pos": (476.65, 1431.80, 'L1')},
            "deliveryRobot_3": {"fleet": "deliveryRobot", "pos": (1153.57, 887.20, 'L1')},
            "tinyRobot_1": {"fleet": "tinyRobot", "pos": (2034.14, 401.16, 'L2')},
            "tinyRobot_2": {"fleet": "tinyRobot", "pos": (2035.11, 479.95, 'L2')},
            "tinyRobot_3": {"fleet": "tinyRobot", "pos": (1156.28, 1254.69, 'L2')}
        },
        "lifts": [(1600.0, 276.0), (1735.0, 276.0), (194.0, 1537.0), (280.0, 1623.0)],
    },
}

# swarm_view.py reads the latest auction from here
AUCTION_FILE = os.path.join(tempfile.gettempdir(), "swarm_auction.json")

LIFT_TIME = 20.0  # seconds to call, ride and exit a lift
TASK_OVERHEAD = 5.0  # seconds for docking/turning

def calculate_distance(pos1, pos2):
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def travel_time(start, goal, speed, lifts):
    """Straight-line ETA, detouring through the best lift when floors differ."""
    if start[2] == goal[2]:
        return calculate_distance(start, goal) / speed
    if not lifts:
        return math.inf
    detour = min(calculate_distance(start, lift) + calculate_distance(lift, goal) for lift in lifts)
    return detour / speed + LIFT_TIME

def load_robots_from_building(path):
    """Build the bidder list from every robot spawned in the building file."""
    import yaml
    with open(path) as f:
        levels = yaml.safe_load(f)["levels"]
    robots = {}
    for level, data in levels.items():
        for v in data["vertices"]:
            params = v[4] if len(v) > 4 else {}
            if "spawn_robot_name" in params:
                model = params["spawn_robot_type"][1].split("/")[-1]  # e.g. TinyRobot
                fleet = model[0].lower() + model[1:]
                robots[params["spawn_robot_name"][1]] = {"fleet": fleet, "pos": (v[0], v[1], level)}
    return robots

def run_auction(world, robots, target_name, current_time):
    """Every robot that can reach the target bids its ETA. Lowest bid wins."""
    target_pos = world["locations"][target_name]
    bids = {}

    # EVERY robot gets to bid, even if busy!
    for robot, state in robots.items():
        fleet = world["fleets"][state["fleet"]]
        if fleet["reach"] is not None and target_name not in fleet["reach"]:
            print(f"  [----] {robot} can't reach {target_name}, sits this one out")
            continue

        # 1. Calculate time penalty if currently busy
        time_penalty = max(0.0, state["free_after"] - current_time)

        # 2. Calculate travel time from its FUTURE position to the new task
        trip = travel_time(state["pos"], target_pos, fleet["speed"], world["lifts"])

        # 3. The Bid is the total Estimated Time of Arrival (ETA)
        bids[robot] = (time_penalty + trip, trip)

        floor_note = f", via lift to {target_pos[2]}" if state["pos"][2] != target_pos[2] else ""
        if time_penalty > 0:
            print(f"  [BID] {robot} is BUSY. Bids ETA: {bids[robot][0]:.1f}s (Penalty: {time_penalty:.1f}s{floor_note})")
        else:
            print(f"  [BID] {robot} is FREE. Bids ETA: {bids[robot][0]:.1f}s{' (' + floor_note[2:] + ')' if floor_note else ''}")

    if not bids:
        return None, bids
    # Ties go to whichever robot is listed first
    winner = min(bids, key=lambda r: bids[r][0])
    return winner, bids

def interactive_dashboard(world, robots):
    names = list(world["locations"])

    print("\n" + "="*50)
    print(" PREDICTIVE SWARM AUCTION INITIALIZED")
    print(f" Bidders ({len(robots)}): {', '.join(robots)}")
    print(" Press Ctrl+C at any time to exit.")
    print("="*50)

    try:
        while True:
            current_time = time.time()

            print("\nAVAILABLE LOCATIONS:")
            for i, name in enumerate(names, 1):
                print(f"  {i:2}. {name}")
            print("-" * 50)

            target_name = input("Enter destination name or number (or type 'exit'): ").strip()

            if target_name.lower() == 'exit':
                break
            # Waypoint names are case-sensitive in Open-RMF (e.g. patrol_A1)
            if target_name.isdigit() and 1 <= int(target_name) <= len(names):
                matches = [names[int(target_name) - 1]]
            else:
                matches = [name for name in names if name.lower() == target_name.lower()]
            if not matches:
                print("Invalid location. Please choose from the list.")
                continue
            target_name = matches[0]

            print(f"\nAUCTION STARTED: Go to {target_name.upper()}")
            winner, bids = run_auction(world, robots, target_name, current_time)
            if winner is None:
                print("No robot can reach that location.")
                continue

            time.sleep(1)

            print("  ---")
            ranking = sorted(bids, key=lambda r: bids[r][0])
            print("  Ranking: " + " < ".join(f"{r} ({bids[r][0]:.1f}s)" for r in ranking))
            print(f"WINNER: {winner} claims the task!")
            try:
                with open(AUCTION_FILE, "w") as f:
                    json.dump({
                        "target": target_name,
                        "target_pos": world["locations"][target_name],
                        "winner": winner,
                        "bids": {r: round(b[0], 1) for r, b in bids.items()},
                    }, f)
            except OSError:
                pass

            # -R only takes effect together with -F; without the fleet name RMF
            # falls back to its own dispatcher and may pick a different robot.
            command = [
                "ros2", "run", "rmf_demos_tasks", "dispatch_go_to_place",
                "-p", target_name, "-F", robots[winner]["fleet"], "-R", winner, "--use_sim_time",
            ]
            try:
                subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"Dispatching {winner} to {target_name}...")
            except FileNotFoundError:
                print("  'ros2' not found. Is your ROS 2 workspace sourced?")

            # Update the Winner's State Memory for the next auction
            _, trip = bids[winner]
            robots[winner]["pos"] = world["locations"][target_name]
            robots[winner]["free_after"] = max(current_time, robots[winner]["free_after"]) + trip + TASK_OVERHEAD

            print("="*50)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nDashboard Shutting Down.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Swarm auction for the Open-RMF demos")
    parser.add_argument("--map", choices=sorted(MAPS), default="office", help="Which rmf_demos world is running")
    parser.add_argument("--building", help="Path to the world's building.yaml; bidders are read from its robot spawns")
    args = parser.parse_args()

    world = MAPS[args.map]
    robots = load_robots_from_building(args.building) if args.building else world["robots"]
    robots = {name: dict(state, free_after=0) for name, state in robots.items()}
    interactive_dashboard(world, robots)
