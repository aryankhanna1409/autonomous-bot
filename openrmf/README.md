# Open-RMF swarm auction

`auction_house.py` (repo root) runs a bidding ceremony across robots in an
rmf_demos world and dispatches the winner with `dispatch_go_to_place -F <fleet> -R <robot>`.
Every robot that can reach the target bids its ETA: how long until it's free,
plus its travel time. On multi-floor maps a lift detour is added when the robot
is on a different floor. The lowest bid wins.

```bash
python3 auction_house.py                 # office (default)
python3 auction_house.py --map clinic    # clinic
```

Destinations can be typed by name or by their number in the list.

## Clinic: 3 robots per floor (6 total)

The stock clinic spawns `deliveryRobot_1/2` on L1 and `tinyRobot_1/2` on L2.
`rmf_demos_clinic_6bots.patch` adds one robot per floor:

| Robot | Floor | Fleet | Spawns + charges at |
|---|---|---|---|
| deliveryRobot_3 | L1 | deliveryRobot | `L1_sub_waiting_area_5` |
| tinyRobot_3 | L2 | tinyRobot | `L2_sub_waiting_area_4` |

Both are existing parking spots on dead-end spurs with at least 3.7 m clear of
furniture, so no new lanes are needed. This copies how `deliveryRobot_1`
already uses `L1_sub_waiting_area_1` as its charger. The patch also lists both
robots in the fleet configs and the crowd-sim agent list.

The two fleets cover different waypoints. tinyRobot's nav graph only reaches
part of L1, so a robot that can't reach the target sits that auction out.
The robots' own charger waypoints are left out of the destination list.

## Office: 3 robots

`rmf_demos_tinyRobot3.patch` turns the unused dead-end spur waypoint in the
office's lower cubicle aisle (1365, 1120) into `tinyRobot3_charger` and
registers `tinyRobot3` in `config/office/tinyRobot_config.yaml`.

## Apply and run

```bash
cd ~/rmf_ws/src/rmf_demos            # your rmf_demos checkout
git apply /path/to/autonomous-bot/openrmf/rmf_demos_clinic_6bots.patch   # and/or the office one
cd ~/rmf_ws && colcon build --packages-select rmf_demos_maps rmf_demos
source install/setup.bash

ros2 launch rmf_demos_gz clinic.launch.xml        # terminal 1
python3 auction_house.py --map clinic             # terminal 2
```

Rebuild `rmf_demos_maps` after patching. The Gazebo world and nav graphs are
generated from the building file at build time.

## Adding more robots

Spawn the robot in the building file and add it to the fleet config. Then
either add it to the map's `robots` table in `auction_house.py`, or pass
`--building <path to building.yaml>` to read every spawned robot straight
from the map.
