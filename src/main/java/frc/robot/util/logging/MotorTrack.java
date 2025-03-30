package frc.robot.util.logging;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.alerts.AlertsManager;
import java.util.HashMap;

public class MotorTrack {
  public static HashMap<Integer, SparkBase> motorHashMap = new HashMap<>();
  public static HashMap<Integer, String> errorHashMap = new HashMap<>();

  public static void addSpark(int id, SparkBase spark) {
    motorHashMap.put(id, spark);
    AlertsManager.addAlert(
        () -> !(getError(id) == " "  || getError(id) == null), "Spark " + id + " Error: " + getError(id), AlertType.kError);
     }

  public static SparkBase getSparkMax(int id) {
    return motorHashMap.get(id);
  }

  public static void update() {
    for (SparkBase spark : motorHashMap.values()) {
      String errors = " ";
      if (spark.hasActiveFault()) {
        if (spark.getFaults().other) {
          errors = errors + "Other ";
        }
        if (spark.getFaults().motorType) {
          errors = errors + "MotorType ";
        }
        if (spark.getFaults().sensor) {
          errors = errors + "Sensor ";
        }
        if (spark.getFaults().can) {
          errors = errors + "CAN ";
        }
        if (spark.getFaults().temperature) {
          errors = errors + "Temperature ";
        }
        if (spark.getFaults().gateDriver) {
          errors = errors + "GateDriver ";
        }
        if (spark.getFaults().escEeprom) {
          errors = errors + "EscEeprom ";
        }
        if (spark.getFaults().firmware) {
          errors = errors + "Firmware ";
        }
      }
      errorHashMap.put(spark.getDeviceId(), errors);
    }
  }
  public static void clearStickyFaults(){

  }
  public static String getError(int id) {
    return errorHashMap.get(id);
  }
}
