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
