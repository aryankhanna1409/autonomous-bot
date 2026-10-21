#!/usr/bin/env python3
import argparse
import math
import subprocess
import time

LOCATIONS = {
    'pantry': (1990.00, 638.36),
    'lounge': (2438.39, 471.24),
    'supplies': (770.88, 383.43),
    'hardware_2': (2474.55, 885.64),
    'coe': (631.56, 587.89),
    'patrol_A1': (1191.44, 824.46),
    'patrol_A2': (2238.47, 477.12),
    'patrol_B': (943.63, 1273.43),
    'patrol_D1': (1991.12, 812.87),
    'patrol_D2': (1210.54, 365.25)
}

FLEET_NAME = "tinyRobot"

# Spawn (charger) positions in the office map. Any number of robots can bid;
# to add one, spawn it in office.building.yaml, list it in the fleet config,
# and add it here (or pass --building to read spawns from the map directly).
ROBOTS = {
    "tinyRobot1": {"pos": (1232.42, 658.57), "free_after": 0},
    "tinyRobot2": {"pos": (2412.58, 627.50), "free_after": 0},
    "tinyRobot3": {"pos": (1365.11, 1120.36), "free_after": 0}
}

SIMULATED_SPEED = 30.0

def calculate_distance(pos1, pos2):
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def load_robots_from_building(path, level="L1"):
    """Build the bidder list from every robot spawned on the given level."""
    import yaml
    with open(path) as f:
        vertices = yaml.safe_load(f)["levels"][level]["vertices"]
    robots = {}
    for v in vertices:
        params = v[4] if len(v) > 4 else {}
        if "spawn_robot_name" in params:
            robots[params["spawn_robot_name"][1]] = {"pos": (v[0], v[1]), "free_after": 0}
    return robots

def run_auction(target_pos, current_time):
    """Every robot bids its ETA (busy penalty + travel time). Lowest bid wins."""
    bids = {}

    # EVERY robot gets to bid, even if busy!
    for robot, state in ROBOTS.items():

        # 1. Calculate time penalty if currently busy
        time_penalty = max(0.0, state["free_after"] - current_time)

        # 2. Calculate travel time from its FUTURE position to the new task
        travel_time = calculate_distance(state["pos"], target_pos) / SIMULATED_SPEED

        # 3. The Bid is the total Estimated Time of Arrival (ETA)
        bids[robot] = (time_penalty + travel_time, time_penalty, travel_time)

        if time_penalty > 0:
            print(f"  [BID] {robot} is BUSY. Bids ETA: {bids[robot][0]:.1f}s (Penalty: {time_penalty:.1f}s)")
        else:
            print(f"  [BID] {robot} is FREE. Bids ETA: {bids[robot][0]:.1f}s")

    # Ties go to whichever robot is listed first
    winner = min(bids, key=lambda r: bids[r][0])
    return winner, bids

def interactive_dashboard():
    print("\n" + "="*50)
    print(" PREDICTIVE SWARM AUCTION INITIALIZED")
    print(f" Bidders ({len(ROBOTS)}): {', '.join(ROBOTS)}")
    print(" Press Ctrl+C at any time to exit.")
    print("="*50)

    try:
        while True:
            current_time = time.time()

            print("\nAVAILABLE LOCATIONS:")
            print(", ".join(LOCATIONS.keys()))
            print("-" * 50)

            target_name = input("Enter destination (or type 'exit'): ").strip()

            if target_name.lower() == 'exit':
                break
            # Waypoint names are case-sensitive in Open-RMF (e.g. patrol_A1)
            matches = [name for name in LOCATIONS if name.lower() == target_name.lower()]
            if not matches:
                print("Invalid location. Please choose from the list.")
                continue
            target_name = matches[0]

            target_pos = LOCATIONS[target_name]

            print(f"\nAUCTION STARTED: Go to {target_name.upper()}")
            winner, bids = run_auction(target_pos, current_time)

            time.sleep(1)

            print("  ---")
            ranking = sorted(bids, key=lambda r: bids[r][0])
            print("  Ranking: " + " < ".join(f"{r} ({bids[r][0]:.1f}s)" for r in ranking))
            print(f"WINNER: {winner} claims the task!")

            # -R only takes effect together with -F; without the fleet name RMF
            # falls back to its own dispatcher and may pick a different robot.
            command = [
                "ros2", "run", "rmf_demos_tasks", "dispatch_go_to_place",
                "-p", target_name, "-F", FLEET_NAME, "-R", winner, "--use_sim_time",
            ]
            try:
                subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"Dispatching {winner} to {target_name}...")
            except FileNotFoundError:
                print("  'ros2' not found. Is your ROS 2 workspace sourced?")

            # Update the Winner's State Memory for the next auction
            # We add 5 seconds to account for docking/turning time
            _, _, travel_time = bids[winner]
            ROBOTS[winner]["pos"] = target_pos
            ROBOTS[winner]["free_after"] = max(current_time, ROBOTS[winner]["free_after"]) + travel_time + 5.0

            print("="*50)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nDashboard Shutting Down.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Swarm auction for the Open-RMF office demo")
    parser.add_argument("--building", help="Path to office.building.yaml; bidders are read from its robot spawns")
    args = parser.parse_args()
    if args.building:
        ROBOTS = load_robots_from_building(args.building)
    interactive_dashboard()
