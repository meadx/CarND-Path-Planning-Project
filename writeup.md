In main.cpp I just added to functions I wrote in tools.cpp.
I wrote a lot of comments, so I hope the code is readable.
The first function is sensorFusion() and changes the lane if necessary and possible or
changes the velocity if necessary and a lane change is not possible.
The second function planPath() plans a smoth path for my car with the calculated velocity in the best lane.

Criteria: The car is able to drive at least 4.32 miles without incident..
-------------------------------------------------------------------------
My car can drive more than 4.32 miles without incident. Recent incidents where:
1.  Collision when changing from an outer lane to the center lane while another car changes
    to the center lane in the same moment.
2.  Total acceleration higher than 10m/s^2.
These 2 incidents and soltions will be discused in the relevant criteria.

##################################################################################################

Criteria: The car drives according to the speed limit.
------------------------------------------------------
In tools.cpp in planPath() in lines 401 and 402 is the calculation of the distance between 2 following points
in the trajectory. Because the car is every 0.02 secounds at the next point, it is possible to calculate a
max. distance, which is possible to drive a given velocity.
The equation is distance = velocity * time;
The current velocity is calculated in tools.cpp in sensorFusion() in lines 267 to 290.
Velocity will be reduced if the distance to another car running ahead is lower than 5m (line 273)
or lower than 25m and the car running ahead is slower than my car (line 279).
Otherwise the car should go as fast as possible, but not higher than the speed limit.
In line 270 is given the max. velocity. Velocity will be increased in lines 283 and 288 just
when the actual velocity is lower than the max. velocity.
The speed limit is 50 mph and in worst case my set velocity 49.699 mph + 0.224 mph = 49.923 mph.
(49.699 mph is the highest possible velocity lower than 49.7 mph and 0.224 mph is the highest
  possible change per 0.02 secounds -> see the following criteria)

##################################################################################################

Criteria: Max Acceleration and Jerk are not Exceeded.
------------------------------------------------------
Max. Acceleration and Jerk are not Exceeded when the velocity changes not more than 0.224 mph per 0.02 secounds.
This value is set in tools.cpp in sensorFusion() in line 268.
Incident 2 happend only 1 time and was not repeatable. In all other cases the max Acceleration and Jerk were not Exceeded.

##################################################################################################

Criteria: Car does not have collisions.
---------------------------------------
This will be done in tools.cpp in sensorFusion() in 2 different cases:
1. Collision while lane change
2. There is a car in the lane and no lane change is possible
In case 1 I do a check, if a lane change will be save for changing in lines 225 to 230.
I check if there is a car next to my car in the neighbor lane.
This is when the car in the neighbor lane has an s value greater 5 or less 8 than mine.
If there is a car, the distance of this lane is set to 0, so it won't be choosen in lines 257 to 260 as best lane.
Because of incident 1 i changed the check of cars in a lane in line 204, to see a lane change of an other car earlier.
Since then, there where no more incidents because of this reason.
In case 2 i reduce the velocity like mentioned in the criteria of the speed limit above.
25 meters won't be enough to break down, when there is a really slow or staying vehicle,
so this is something I could improve in my code. But in the simulator there aren't really slow cars.

##################################################################################################

Criteria: The car stays in its lane, except for the time between changing lanes.
--------------------------------------------------------------------------------
Each lane has its best d position:
lane 0: d = 2
lane 1: d = 6
lane 2: d = 10
These values can be calculated by the formular 2 + 4 * lane.
So my main waypoints will be calculated dependent to the best lane in tools.cpp in planPath()
in lines 348 to 353.

##################################################################################################

Criteria: The car is able to change lanes
-----------------------------------------
This will be done in tools.cpp in sensorFusion() in lanes 236 to 263.
If the distance to a car running ahead in the current lane is not lower than 30 m
the car will stay in the current lane (line 246 to 250).
Otherwise I check, which neighbor lane has a higher distance than the current lane and
the highest distance to a car running ahead, if there are 2 possibilities (lines 251 to 262).
If a change is not secure the distance of this lane won't be choosen, because it was set to 0.
(see criteria: Car does not have collisions)
