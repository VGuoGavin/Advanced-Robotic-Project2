# UR3 robot arm: Kinematics, Dynamics and Control

UR3 is a 6-degree-of-freedom (DoF) robotic manipulator manufactured by Universal Robots Company.

The robot arm kinematic model is built according to the Denavit-Hartenberg convention. A Decentralized PID Controller and a Centralized Inverse Dynamics Controller was designed. Finally, a trajectory is performed and the performance/results of both controllers compared and analysed.

<p float="left">
  <img src="https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/ur3.png" width="300" />
  <img src="https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/ur3%20simplified%20model.png" width="300" />
</p>

Links: \[ [Centralized simulink model (preview)](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/Centralized%20Simulink%20Model.png) \]
\[ [Decentralized simulink model (preview)](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/Decentralized%20Simulink%20Model.png) \]

## Results


In order to evaluate the designed controllers and the compare a trajectory for the end effector was created, which consisted in performing a capital "L" followed by a capital "C", combining both rectilinear and circular trajectories.

**Trajectory Characteristics:**
Joint angles             |  Torque
:-------------------------:|:-------------------------:
![](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/trajectory%20joint%20angles.png)  |  ![](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/trajectory%20torque.png)

**Centralized Controller Analysis:**
Position Error             |  Orientation Error
:-------------------------:|:-------------------------:
![](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/Centralized%20controller%20-%20Absolute%20position%20error%20.png)  |  ![](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/Centralized%20controller%20-%20absolute%20orientation%20error.png)

**Decentralized Controller Analysis:**
Position Error             |  Orientation Error
:-------------------------:|:-------------------------:
![](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/Decentralized%20controller%20-%20absolute%20position%20error.png)  |  ![](https://github.com/luis-a-miranda/UR3-robot-arm-Kinematics-Dynamics-and-Control/blob/main/images/Decentralized%20controller%20-%20absolute%20orientation%20error.png)


The **maximum error** obtained with **centralized control** was approximately **0.00121 m and 0.097 rad for position and orientation**, respectively. With **decentralized controller** the maximum errors obtained were **0.036 m and 0.1 rad**, having a worse position error than the previous and a very similar orientation error.

The same **trajectory is performed at higher speed** to ensure the results are valid **and the centralized controller has a better performance** once again.


### Conclusion

As this report demonstrates, **the dynamic model, with the Newton-Euler formulation, was successfully built and validated** by two tests. One with the manipulator arm at rest and another by making the manipulator arm behave as a gravitational pendulum. Regarding the **controllers**, both **were successfully designed and implemented**. The **decentralized controller wasn't as successful as the centralized one for high velocity trajectories and also as a higher position and pose errors**, but more significant in the position one. From this, **the best controller is the centralized one**.
