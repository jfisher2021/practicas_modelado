

https://github.com/jfisher2021/practicas_modelado/assets/113594937/4adc3805-dd69-4a17-8687-88b988f3ab0b

Observations not seen in the video: The value of the controller varies according to the inclination. When the root is going up the ramp, the motor velocity varies depending on the time the robot is going up the ramp. There is a variable that increases as the robot spends more time going up the ramp, and this variable is multiplied by 6.5 to increase the motor velocity. The motor velocity keeps increasing until the robot reaches the top of the ramp, and when the robot reaches the top of the ramp, the motor velocity is 11.2. When going down the ramp, the motor velocity is 8.9, and when on the flat surface, the motor velocity is 11.2.

The value of the force also varies according to the inclination. When the root is going up the ramp, the motor force is 25, it could be higher but I didn't see the need to increase it as it didn't change the robot's behavior significantly. The most important thing is that the force is applied to the motors of the rear wheels, so that it provides enough force for the robot to climb the ramp. If it is applied to the motors of the front wheels, the robot would not be able to climb the ramp. It's also important to note that if too much force is applied, the robot tilts too much and falls, which is why the force is set to 25. When going down the ramp, the motor force is 75 to brake faster and prevent the robot from falling, and when on the flat surface, the motor force is 25.

```python

best_velocity = 11.2
velDown = 8.9
force = 25
num = 0


 if (angle_degrees > -39 and angle_degrees < -10):
                         num += 1  
                         p.setJointMotorControlArray(terreneitorModel, [4, 5], p.VELOCITY_CONTROL, targetVelocities=[i * 6.5] * 2, forces=[force] * 2)
                  elif (angle_degrees > 2):
                         p.setJointMotorControlArray(terreneitorModel, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[velDown] * 4, forces=[force * 3] * 4)
                  else:
                         p.setJointMotorControlArray(terreneitorModel, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=[best_velocity] * 4, forces=[force] * 4)
```