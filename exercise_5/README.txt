EXERCISE 5:
Generare tre tartarughe. Controllare la prima dal punto iniziale ad un punto dato finale con un controllo a 50 hz evitando la collisione con le altre due (ferme).
Testare l'algoritmo in varie configurazioni delle tarturghe "ostacolo".

Execution:
1) default scenario: roslaunch exercise_5 exercise_5_v1.launch
   the robot safely avoids the obstacles and get to the goal
2) stronger obstacles: roslaunch exercise_5 exercise_5_v2.launch
   the robot keeps more distance from the obstacle
3) local minima: roslaunch exercise_5 exercise_5_v3.launch
   the robot gets stuck, it is a common problem in potentials based motion planning
