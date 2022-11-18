#!/usr/bin/env bash
tasks='input_files/task_static_patrol_no_currents.py input_files/task_static_patrol_no_divide.py input_files/task_static_patrol.py input_files/task_static_patrol_obstacles.py input_files/task_coordinated_patrol_step2_patrol_online_assignment.py input_files/task_area_coverage_step2_patrol_online_assignment.py input_files/task_tracking_a_target.py input_files/task_tracking_multiple_targets.py input_files/task_multiple_ships_tracking_a_target.py input_files/task_multiple_ships_tracking_multiple_targets.py'

for f in $tasks; do  pythonw main.py -f $(basename $f) ; done
