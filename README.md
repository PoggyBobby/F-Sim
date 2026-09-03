# fsae-sim-hand

## About
This repo serves as the bringup software the first three revisions of torque vectoring. The milestones go as the following

1. Rear 2-motor S-diff
    Using ackerman geometry to control the inside wheels to reduce speed on a turn
2. 4-motor S-diff
3. Yaw-rate TV
    Using a PI controller to optimize the yaw moment of the vehicle
4. Traction/load-aware TV


## Directory Structure
```
tests/
docs/
examples/
model/
    physical/
        tires/ (copy over)
    
    
    sensors/
        wheel_speed/
        steering_angle/
        throttle_pos/
        brake_pressure_sens/

        imu_6axis/
        gps/



controllers/


```
