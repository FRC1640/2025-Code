# Constants

## Table of Contents
- [Constants](#constants)
  - [Table of Contents](#table-of-contents)
  - [Robot Switching](#robot-switching)
    - [RobotSwitch](#robotswitch)
    - [RobotSwitchManager](#robotswitchmanager)
    - [Alternative Legacy](#alternative-legacy)

## Robot Switching

To create switchable robot variables, use the following methods:

### RobotSwitch

This is the recommended method. In `RobotConstants.java`, define a switchable robot variable as follows:

```java
public static final <VALUE> variable =
                new RobotSwitch<VALUE>(<Default Value>)
                        .addValue(<RobotType>, <Value for this Robot Type>)
                        .get();
```

Example:

```java
public static final int integer =
                new RobotSwitch<Integer>(2)
                        .addValue(RobotType.Deux25, 3)
                        .addValue(RobotType.Deux24, 5)
                        .get();
```

If the robot is `Deux25`, it returns `3`; if it is `Deux24`, it returns `5`. If it isn't either, it returns 2

### RobotSwitchManager

Alternatively, use `RobotSwitchManager` for a more memory-efficient solution, though it may be less reliable:

```java
public static final <Type> variable =
                RobotSwitchManager.robotTypeValue(
                        <Default Value>,
                        new RobotTypeParm<Type>(<RobotType>, <Value>),
```

Example:

```java
public static final boolean liftSubsystemEnabled =
                RobotSwitchManager.robotTypeValue(
                        true,
                        new RobotTypeParm<Boolean>(RobotType.Deux25, false),
                        new RobotTypeParm<Boolean>(RobotType.Deux24, true));
```
If it is Deux25 robot, it returns false. If it is Deux24, it returns true. If it isn't either, it returns the default value of true
### Alternative Legacy
Another method:
```java
public static final <Type> variable =
                RobotSwitchManager.robotTypeValue(<deux25 value>, <prime25 value>, <duex24 value>, <prime24 value>);
```

This method is less frequently updated and not recommended. It takes up the least memory.
