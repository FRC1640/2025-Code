package frc.robot.util.logging;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import java.util.HashMap;

public class MotorTrack {
  public static HashMap<Integer, SparkBase> motorHashMap = new HashMap<>();

  public static void addSpark(int id, SparkMax spark) {
    motorHashMap.put(id, spark);
  }

  public static void addSpark(int id, SparkFlex spark) {
    motorHashMap.put(id, spark);
  }

  public static SparkBase getSparkMax(int id) {
    return motorHashMap.get(id);
  }
}
