# Motor Limits/Clamping voltage
Import this class:

```java
import frc.robot.util.tools.MotorLim;
```

### Applying Limits
```java
MotorLim.applyLimits(double pos (encoder pos), double voltage (get voltage of motor), Limit limit (the limits of the motor you set))
```
Limits are constructed like this:
```java
public static final Limit limits = new Limit(low, high);
```
### Clamp Voltage
When setting the voltage of a motor, use this method to prevent the voltage from going from -12 - 12

```java
spark.setVoltage(MotorLim.clampVoltage(11));
```
this sets the voltage to 11 on the motor, but if i do
```java
spark.setVoltage(MotorLim.clampVoltage(200));
```
It does not overvolt the motor, but instead sets it to 0 voltage. So if it suddenly stops, there may be something wrong with your code, not the motors.
