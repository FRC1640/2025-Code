# RevLib Spark Configurer

---
## **Table of Contents**
- [RevLib Spark Configurer](#revlib-spark-configurer)
  - [**Table of Contents**](#table-of-contents)
    - [Making your motor](#making-your-motor)
    - [Follower Motors](#follower-motors)
  - [Config:](#config)
      - [configSparkMax](#configsparkmax)
      - [SparkConfiguration:](#sparkconfiguration)
    - [PID Controller](#pid-controller)
      - [Construct the Spark PID](#construct-the-spark-pid)

---
### Making your motor
```java
SparkMax motor1;
motor1 =
SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(<motor id>, <is inverted (true/false)>));
```
So if I wanted to make a regular spinning motor with the ID of 1, i would do:

```java
SparkMax motor1;
motor1 = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(1, false));
```
And if I wanted to make a inverted motor with the ID of 1, I would do:

```java
SparkMax motor1;
motor1 = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(1, true));
```

For more, check out the classes here:

[Jake's Spark Config](https://github.com/FRC1640/2025-Code/tree/master/src/main/java/frc/robot/util/spark/SparkConfiguration.java)

[Jake's Spark Configurer](https://github.com/FRC1640/2025-Code/tree/master/src/main/java/frc/robot/util/spark/SparkConfigurer.java)
### Follower Motors
A follower motor is a motor that copies the same voltage/speed that is applied to the leader motor.

__SETTINGS ARE NOT APPLIED TO THE FOLLOWER FROM THE LEADER MOTOR__
```java
leaderMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftleaderMotorID, false));

followerMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(LiftConstants.liftfollowerMotorID, false), leaderMotor);
```

## Config:
#### configSparkMax
```java
configSparkMax(SparkConfiguration config, (optional) SparkMax leader)
```
#### SparkConfiguration:

```java
SparkConfiguration(
      int id,
      IdleMode idleMode,
      boolean inverted,
      int currentLimit,
      int encoderMeasurementPeriod,
      int encoderAverageDepth,
      StatusFrames statusFrames,
      PIDConstants pid,
      LimitSwitchConfig limitSwitch,
      SparkBaseConfig seed)
```
### PID Controller
In RobotPIDConstants.java, create a new SparkPIDConstant like this:
```java
public static final SparkPIDConstants pidConstantSpark =
      new SparkPIDConstants(kP, kI, kD, ClosedLoopSlot);
```
There are command decorators you can add to it, as follows:
```java
/*
   * Set the constraints of output of the PID
   */
  .setConstraint(double minOutput, double maxOutput)
  /*
   * Sets the Velocity Feed Forward
   */
  .setVelocityFF(double velocityFF)
  /*
   * Sets max velocity
   */
  .setMaxVelocity(double maxVel)
  /*
   * Set Max Acceleration
   */
  .setMaxAccel(double maxAccel)
  /*
   * Set Allowed Error
   */
  .setAllowedErr(double allowedErr)
  /*
   * Set the MAXPosition mode
   */
  .setMaxPositionMode(MAXMotionPositionMode maxPositionMode)
  /*
   * Set the Closed Loop Slot on the Spark
   */
  .setClosedLoopSlot(ClosedLoopSlot closedLoopSlot)

  /*
   * Set Position Conversion Factor 
   */

  .setPositionConversion(Double conversion)

  /* 
   * Set Velocity Conversion Factor 
   */
  .setVelocityConversion(Double conversion)
```

To apply them, do like follows:
```java
  public static final SparkPIDConstants pidConstantSpark =
      new SparkPIDConstants(0, 0, 0, ClosedLoopSlot.kSlot0).commandDecorator(parameter).commandDecorator2(parameter);
```
#### Construct the Spark PID
On your motor, do this:
```java
public static final SparkPIDConstants variable =
      new SparkPIDConstants(0, 0, 0, ClosedLoopSlot.kSlot0).commandDecorator(parameter);
leaderMotor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftleaderMotorID, false)
                .applyPIDConfig(variable));
```
For actually detailed documentation, check out here:
[**Spark PID/Closed Loop Documentation**](https://docs.revrobotics.com/revlib/spark/closed-loop)
[**MAXMotion Documentation**](https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control)
