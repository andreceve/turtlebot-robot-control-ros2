# 2 · TurtleBot3 perception, control and path planning

Second assignment: using the LiDAR to drive autonomously along a track, and implementing the
A\* search algorithm.

## Part 1 — LiDAR-based autonomous driving (`Part1/exercise4`)

**Task 1** — visualise the LiDAR scan of the robot in RViz (`Part1/task1_lidar.png`).

**Task 2** — drive the robot autonomously along a narrow zig-zag track with an obstacle at the
end, using only the LiDAR (`scripts/drive.py`).

How the controller works:

- **Steering (PD control).** For each scan I take the closest obstacle on the left
  (15°–90°) and on the right (270°–345°). The error is `left − right`, and a PD law
  (`Kp = 1.7`, `Kd = 0.9`) gives the angular velocity, saturated at ±0.9 rad/s to avoid
  over-steering into the walls. Wide sectors make the controller react to the closest
  protruding wall, so the robot stays centred through the zig-zag.
- **Speed.** The robot drives at 0.12 m/s and slows down to 0.06 m/s when something is closer
  than 0.5 m in the front cone (±15°), which gives room to steer around the final pillar.
- **End of track.** When the whole front half (−90° to +90°) is free beyond 3 m, the robot is in
  the open area at the end of the track and stops.

A screen recording of the run is in `Part1/task2_turtlebot_driving.mp4`.

```bash
colcon build --packages-select exercise4 && source install/setup.bash
ros2 launch exercise4 exercise4.launch.py   # Gazebo world with the track
ros2 run exercise4 drive.py                 # in a second terminal
```

The track model (`exercise4_track`) was provided with the course and is not included.

## Part 2 — A\* search (`Part2/Exercise4_Part2_Astar.ipynb`)

Jupyter notebook in which I implemented A\* on a grid maze, visualised with pygame:

- **Heuristic** — Manhattan distance.
- **Search** — a neighbour is kept if it is new or reached with a lower cost; its cost and
  parent are stored and it is added to the frontier.
- **Path** — rebuilt by walking the `came_from` links back from the goal.

Results saved in the notebook:

| Maze | Path length | Cells visited |
|---|---|---|
| 15 × 15 | 18 steps | 35 |
| 41 × 41 | 424 steps | 703 |

Effect of the heuristic on a small maze:

| Estimate | Cells visited | Path length |
|---|---|---|
| none (blind search) | 140 | 32 |
| Manhattan | 53 | 32 |
| 3 × Manhattan | 40 | 36 |

A good (admissible) heuristic visits far fewer cells and still finds the shortest path; an
over-estimating one is faster but returns a longer path. The notebook also contains my answers
to the reflection questions.
