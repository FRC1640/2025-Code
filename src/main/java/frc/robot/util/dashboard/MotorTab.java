package frc.robot.util.dashboard;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotConstants.MotorInfo;
import frc.robot.util.logging.MotorTrack;
import java.util.Map;

public class MotorTab {
  public ShuffleboardTab motorTab;
  private static SendableChooser<SparkBase> motorSelect = new SendableChooser<SparkBase>();
  public SparkBase selectedSparkMax;
  NetworkTable networkTable;

  public void init() {
    motorTab = Shuffleboard.getTab("Max Motor Testing");
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    networkTable = nt.getTable("/Shuffleboard/Max Motor Testing");
    createWidgets();
  }

  public void createWidgets() {
    motorSelect.setDefaultOption("none", null);
    for (int id : MotorTrack.motorHashMap.keySet()) {
      if (MotorInfo.motorLoggingManager.getMap().containsKey(id)) {
        motorSelect.addOption(
            MotorInfo.motorLoggingManager.getMap().get(id), MotorTrack.motorHashMap.get(id));
      } else {
        motorSelect.addOption("Motor " + id, MotorTrack.motorHashMap.get(id));
      }
    }
    motorSelect.onChange((x) -> update());
    motorTab.add(motorSelect).withSize(3, 1).withPosition(0, 0);
    GenericEntry voltageSet =
        motorTab
            .add("Voltage Set", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12))
            .withPosition(0, 1)
            .withSize(9, 1)
            .getEntry();
    motorTab
        .add(
            "Run Spark At Voltage",
            new InstantCommand(
                () -> {
                  if (selectedSparkMax != null) {
                    selectedSparkMax.setVoltage(networkTable.getEntry("Voltage Set").getDouble(0));
                  }
                }))
        .withPosition(0, 2);
    motorTab
        .add(
            "Stop Spark",
            new InstantCommand(
                () -> {
                  if (selectedSparkMax != null) {
                    selectedSparkMax.setVoltage(0);
                  }
                }))
        .withPosition(0, 3);
  }

  public void update() {
    selectedSparkMax = motorSelect.getSelected();
  }
}
