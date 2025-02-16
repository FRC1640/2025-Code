package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.logging.TrackedRobotPID.PIDTrack;

public class PIDTab {
  private static SendableChooser<String> pidChooser = new SendableChooser<String>();
  public static ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable = nt.getTable("/Shuffleboard/PID Tuning");

  GenericEntry kPSet;
  GenericEntry kISet;
  GenericEntry kDSet;

  public PIDTab() {}

  public void init() {
    widgetBuild();
  }

  public void widgetBuild() {
    pidTunerBuild();
  }

  public void pidTunerBuild() {
    pidChooser.setDefaultOption("none", "empty");
    for (String name : PIDTrack.pidsTrack.keySet()) {
      pidChooser.addOption(name, name);
    }
    pidChooser.onChange((x) -> updateNetworkTable(PIDTrack.pidsTrack.get(x)));
    pidTab.add(pidChooser).withSize(3, 1).withPosition(0, 1);

    kPSet = pidTab.add("kP", 0).withPosition(0, 0).getEntry();
    kISet = pidTab.add("kI", 0).withPosition(1, 0).getEntry();
    kDSet = pidTab.add("kD", 0).withPosition(2, 0).getEntry();
    pidTab
        .add(
            "Set Values",
            new InstantCommand(
                () -> {
                  PIDController constructController =
                      PIDTrack.pidsTrack.get(pidChooser.getSelected());
                  constructController.setP(kPSet.getDouble(0));
                  constructController.setI(kISet.getDouble(0));
                  constructController.setD(kDSet.getDouble(0));
                  PIDTrack.pidsTrack.put(pidChooser.getSelected(), constructController);
                }))
        .withPosition(3, 0);
  }

  public void updateNetworkTable(PIDController select) {
    networkTable.getEntry("kP").setDouble(select.getP());
    networkTable.getEntry("kI").setDouble(select.getI());
    networkTable.getEntry("kD").setDouble(select.getD());
  }
}
