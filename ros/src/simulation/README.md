# Simulation

The Simulation package launches the Gazebo simulator.

## Run

To launch the Gazebo simulator, run:

```
ros2 launch simulation simulation_launch.py n_robots:=1 robot_offset:=1.0 robot_arrangement:=CIRCLE
```

| **Parameter**     | **Required** | **Description**                                           | **Type**                 | **Default** |
| ----------------- | ------------ | --------------------------------------------------------- | ------------------------ | ----------- |
| n_robots          | ❌           | The number of robots to spawn in the simulation           | `int`                    | 1           |
| robot_offset      | ❌           | The closest distance in meters between each spawned robot | `float`                  | 1.0         |
| robot_arrangement | ❌           | The pattern of the spawned robots                         | `LINE`, `GRID`, `CIRCLE` | `LINE`      |

## Details

Under the hood, this package generates a new `sdf` file from templates at launch time. The templates are located in the `description/` folder, which also contains a `yaml` file with easily modifiable robot parameters. `sdf` files do not support variables, so a custom Python script is used to substitute `{% VARIABLES %}` with their values.
