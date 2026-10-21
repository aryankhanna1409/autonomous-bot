# Open-RMF office demo: 3-robot auction

`auction_house.py` (repo root) runs a bidding ceremony over every robot in the
`tinyRobot` fleet of the rmf_demos **office** world and dispatches the winner.

The stock office world only spawns `tinyRobot1` and `tinyRobot2`.
`rmf_demos_tinyRobot3.patch` adds a third one:

- `rmf_demos_maps/maps/office/office.building.yaml`: the existing dead-end
  spur waypoint in the lower cubicle aisle (x=1365, y=1120) becomes
  `tinyRobot3_charger` and spawns `tinyRobot3`. No new lanes are needed.
- `rmf_demos/config/office/tinyRobot_config.yaml`: registers `tinyRobot3`
  with that charger.

## Apply and run

```bash
cd ~/rmf_ws/src/rmf_demos            # your rmf_demos checkout
git apply /path/to/autonomous-bot/openrmf/rmf_demos_tinyRobot3.patch
cd ~/rmf_ws && colcon build --packages-select rmf_demos_maps rmf_demos
source install/setup.bash

ros2 launch rmf_demos_gz office.launch.xml        # terminal 1
python3 auction_house.py                          # terminal 2
```

The map must be rebuilt (`rmf_demos_maps`) because the Gazebo world and nav
graph are generated from the building file at build time.

## More than 3 robots

The auction loops over every entry in `ROBOTS`, so N robots works the same
way. Spawn another robot in the building file, add it to the fleet config,
then either add it to `ROBOTS` or run
`python3 auction_house.py --building <path to office.building.yaml>` to read
every spawned robot straight from the map.
