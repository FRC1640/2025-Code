package frc.robot.util.logging;

import java.util.HashMap;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

public class MotorTrack {
  public static HashMap<Integer, SparkMax> motorMaxHashMap = new HashMap<>();
  public static HashMap<Integer, SparkFlex> motorFlexHashMap = new HashMap<>();

}
