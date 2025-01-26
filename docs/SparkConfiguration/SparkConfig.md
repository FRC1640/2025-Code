# RevLib Spark Configurer
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
