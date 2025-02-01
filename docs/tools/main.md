# **Tools**
---
## **Table of Contents**
- [**Tools**](#tools)
  - [**Table of Contents**](#table-of-contents)
  - [**Alliance Manager**](#alliance-manager)
    - [**Import:**](#import)
    - [**Choose Values Based on Alliance**](#choose-values-based-on-alliance)
  - [**Motor Limits \& Voltage Clamping**](#motor-limits--voltage-clamping)
    - [**Import:**](#import-1)
    - [**Applying Limits to Motors**](#applying-limits-to-motors)
    - [**Clamping Motor Voltage**](#clamping-motor-voltage)
  - [**Distance Manager**](#distance-manager)
    - [**Import:**](#import-2)
    - [**Finding the Nearest Position**](#finding-the-nearest-position)
  - [Robot Type Switcher](#robot-type-switcher)
    - [Variable Switcher for Robots](#variable-switcher-for-robots)
---
## **Alliance Manager**
### **Import:**
```java
import frc.robot.util.tools.AllianceManager;
```

### **Choose Values Based on Alliance**
```java
AllianceManager.chooseFromAlliance(T valueBlue, T valueRed)
```
- Returns a value based on the current alliance color.
- If the robot is on the **blue alliance**, it returns `valueBlue`.
- If the robot is on the **red alliance**, it returns `valueRed`.
- Supports any data type.

---

## **Motor Limits & Voltage Clamping**
### **Import:**
```java
import frc.robot.util.tools.MotorLim;
```

### **Applying Limits to Motors**
```java
MotorLim.applyLimits(double pos, double voltage, Limit limit)
```
- Prevents the motor from exceeding predefined position limits.
- **Limits** are created as follows:
  ```java
  public static final Limit limits = new Limit(double low, double high);
  ```

### **Clamping Motor Voltage**
- Ensures motor voltage stays within the safe range (-12V to 12V).
- Example usage:
  ```java
  spark.setVoltage(MotorLim.clampVoltage(11)); // Sets voltage to 11V
  spark.setVoltage(MotorLim.clampVoltage(200)); // Clamped to prevent from going past 12V
  ```

---

## **Distance Manager**
### **Import:**
```java
import frc.robot.util.tools.DistanceManager;
```

### **Finding the Nearest Position**
```java
DistanceManager.getNearestPosition(Pose2d robotPos, Pose2d[] checkPoints)
```
- Finds the closest point to `robotPos` from an array of checkpoints (`checkPoints`).
- Useful for pathfinding and navigation.

## Robot Type Switcher
In [RobotConstants.java](/src/main/java/frc/robot/constants/RobotConstants.java), in the RobotConfigConstants class, set the Robot Type in the robotType variable. There are the following options:
- Duex25
- Prime25
Duex25 is the setting if the robot is the Duex 2025 robot, and Prime25 is Prime 2025 robot.
### Variable Switcher for Robots
You can set any variable like this using the switcher:
First, import this class:
```java
import frc.robot.util.tools.RobotSwitch;
```
Then, set the variable like this
```java
boolean value = RobotSwitch.robotTypeValue(true, false);
```
If the robot is Duex 2025, it will return true, and if it is Prime 2025, it will return false
The boolean can be replaced by any data type, so it can also be:
```java
int value = RobotSwitch.robotTypeValue(1,2);
```
If the robot is Duex 2025, it will return 1, and if the robot is Prime 2025, it will return 2. 