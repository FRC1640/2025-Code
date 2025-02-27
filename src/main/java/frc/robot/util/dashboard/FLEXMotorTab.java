package frc.robot.util.dashboard;

import com.revrobotics.spark.SparkFlex;
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

public class FLEXMotorTab {
  public ShuffleboardTab motorTab;
  private static SendableChooser<SparkFlex> motorSelect = new SendableChooser<SparkFlex>();
  public SparkFlex selectedSparkFlex;
  NetworkTable networkTable;

  public void init() {
    motorTab = Shuffleboard.getTab("Flex Motor Testing");
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    networkTable = nt.getTable("/Shuffleboard/Flex Motor Testing");
    createWidgets();
  }

  public void createWidgets() {
    motorSelect.setDefaultOption("none", null);
    for (int id : MotorTrack.motorFlexHashMap.keySet()) {
      if (MotorInfo.motorLoggingManager.getMap().containsKey(id)) {
        motorSelect.addOption(
            MotorInfo.motorLoggingManager.getMap().get(id), MotorTrack.motorFlexHashMap.get(id));
      } else {
        motorSelect.addOption("Motor " + id, MotorTrack.motorFlexHashMap.get(id));
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
                  if (selectedSparkFlex != null) {
                    selectedSparkFlex.setVoltage(networkTable.getEntry("Voltage Set").getDouble(0));
                  }
                }))
        .withPosition(0, 2);
    motorTab
        .add(
            "Stop Spark",
            new InstantCommand(
                () -> {
                  if (selectedSparkFlex != null) {
                    selectedSparkFlex.setVoltage(0);
                  }
                }))
        .withPosition(0, 3);
  }

  public void update() {
    selectedSparkFlex = motorSelect.getSelected();
  }
}
