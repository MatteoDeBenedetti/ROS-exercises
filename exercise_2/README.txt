roslaunch exercise_2 exercise_2.launch lin_vel_sat_min:=-3 lin_vel_sat_max:=3 ang_vel_sat_min:=-1 ang_vel_sat_max:=1

rostopic pub -r 10 /custom_control exercise_2/CustomControl "{lin_vel_ref: 4.0, ang_vel_ref: 2.0}"
