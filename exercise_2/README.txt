EXERCISE 2:
Scrivere un nodo che simuli un controllore di alto livello che, presa in ingresso una richiesta di controllo di tipo custom (contenente velocità lineare longitudinale e velocità angolare sullo yaw), generi un controllo per la tartaruga di tipo opportuno che applichi una saturazione sulla richiesta di controllo. I valori della saturazione devono essere impostabili come parametro del nodo.
Dimostrare l'effettivo funzionamento del nodo confrontando la richiesta di controllo e il controllo effettuato su rqt_plot.

Execution:
roslaunch exercise_2 exercise_2.launch lin_vel_sat_min:=-3 lin_vel_sat_max:=3 ang_vel_sat_min:=-1 ang_vel_sat_max:=1

To send custom commands typein another terminal:
rostopic pub -r <value> /custom_control exercise_2/CustomControl "{lin_vel_ref: <value>, ang_vel_ref: <value>}"
eg: rostopic pub -r 10 /custom_control exercise_2/CustomControl "{lin_vel_ref: 4.0, ang_vel_ref: 2.0}"

Visualization:
rqt_plot /turtle1/cmd_vel/linear/x /turtle1/cmd_vel/angular/z /custom_control

