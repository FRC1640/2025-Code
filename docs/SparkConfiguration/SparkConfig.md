# Spark Configurer Class
### Making your motor
```
SparkMax motor1;
motor1 =
SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(<motor id>, <is inverted (true/false)>));
```
So if I wanted to make a regular spinning motor with the ID of 1, i would do:
```
SparkMax motor1;
motor1 = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(1, false));
```
And if I wanted to make a inverted motor with the ID of 1, I would do:
```
SparkMax motor1;
motor1 = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(1, true));
```

For more, check out the classes here:

[Jake's Spark Config](/src/main/java/frc/robot/util/spark/SparkConfiguration.java)
### Followers
WIP
