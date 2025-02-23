package frc.robot.util.logging;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import java.util.HashMap;

public class MotorTrack {
  public static HashMap<Integer, SparkMax> motorMaxHashMap = new HashMap<>();
  public static HashMap<Integer, SparkFlex> motorFlexHashMap = new HashMap<>();

  public static void addSpark(int id, SparkMax spark) {
    motorMaxHashMap.put(id, spark);
  }

  public static void addSpark(int id, SparkFlex spark) {
    motorFlexHashMap.put(id, spark);
  }

  public static SparkMax getSparkMax(int id) {
    return motorMaxHashMap.get(id);
  }

  public static SparkFlex getSparkFlex(int id) {
    return motorFlexHashMap.get(id);
  }
}
