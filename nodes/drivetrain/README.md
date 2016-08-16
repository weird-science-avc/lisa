# drivetrain

```
go run nodes/drivetrain/*.go
```

The drivetrain is responsible for controlling the speed and steering of
the car.

#### Subscribes
* `/lisa/cmd_steering` - Target steering from -1.0 (right) and 1.0 (left).
* `/lisa/cmd_velocity` - Target speed in MPH.

#### Publishes
N/A
