# Motor Limits/Clamping voltage
Import this class:

```java
import frc.robot.util.tools.MotorLim;
```

### Applying Limits
```java
MotorLim.applyLimits(double pos, double voltage, Limit limit)
```
This prevents the motor from going past the set limits.
Limits are constructed like this:
```java
public static final Limit limits = new Limit(double low, double high);
```
### Clamp Voltage
When setting the voltage of a motor, use this method to clamp the voltage between -12 and 12.

```java
spark.setVoltage(MotorLim.clampVoltage(11));
```
this sets the voltage to 11 on the motor, but if you do
```java
spark.setVoltage(MotorLim.clampVoltage(200));
```
It does not overvolt the motor, but instead sets it to 12 volts.
