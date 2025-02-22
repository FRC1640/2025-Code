package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.dashboard.PIDInfo.PIDCommandRegistry;
import frc.robot.util.logging.PIDTracking.PIDTrack;

public class PIDTab {
  private static SendableChooser<PIDController> pidChooser = new SendableChooser<PIDController>();
  public static ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable = nt.getTable("/Shuffleboard/PID Tuning");

  GenericEntry kPSet;
  GenericEntry kISet;
  GenericEntry kDSet;
  GenericEntry iZoneSet;
  GenericEntry tolerance;

  GenericEntry setpoint;

  public PIDTab() {}

  public void init() {
    pidTunerBuild();
  }

  public void pidTunerBuild() {
    pidChooser.setDefaultOption("none", new PIDController(0, 0, 0));
    for (String info : PIDTrack.pidsTrack.keySet()) {
      pidChooser.addOption(info, PIDTrack.pidsTrack.get(info));
    }
    pidChooser.onChange((x) -> updateNetworkTable(x));
    pidTab.add(pidChooser).withSize(3, 1).withPosition(0, 0);

    kPSet = pidTab.add("kP", 0).withPosition(0, 1).getEntry();
    kISet = pidTab.add("kI", 0).withPosition(1, 1).getEntry();
    kDSet = pidTab.add("kD", 0).withPosition(2, 1).getEntry();
    iZoneSet = pidTab.add("IZone", 0).withPosition(3, 1).getEntry();
    tolerance = pidTab.add("Tolerance", 0).withPosition(4, 1).getEntry();
    setpoint = pidTab.add("Set", 0).withPosition(0, 2).withSize(1, 1).getEntry();
    pidTab
        .add(
            "Run Attached PID Command",
            new InstantCommand(
                () -> {
                  if (PIDCommandRegistry.attachedCommands.get(pidChooser.getSelected()) != null) {
                    if (PIDCommandRegistry.currentlyRunningCommand != null) {
                      CommandScheduler.getInstance()
                          .cancel(PIDCommandRegistry.currentlyRunningCommand);
                    }

                    PIDCommandRegistry.currentlyRunningCommand =
                        PIDCommandRegistry.attachedCommands
                            .get(pidChooser.getSelected())
                            .apply(networkTable.getEntry("Set").getDouble(0));
                    PIDCommandRegistry.currentlyRunningCommand.schedule();
                  }
                }))
        .withPosition(1, 2);
    pidTab
        .add(
            "Set Values",
            new InstantCommand(
                () -> {
                  System.out.println("starting");
                  pidChooser.getSelected().setP(kPSet.getDouble(0));
                  pidChooser.getSelected().setI(kISet.getDouble(0));
                  pidChooser.getSelected().setD(kDSet.getDouble(0));
                  pidChooser.getSelected().setIZone(iZoneSet.getDouble(0));
                  pidChooser.getSelected().setTolerance(tolerance.getDouble(0));
                  System.out.println("ending");
                }))
        .withPosition(5, 1);
  }

  public void updateNetworkTable(PIDController select) {
    networkTable.getEntry("kP").setDouble(select.getP());
    networkTable.getEntry("kI").setDouble(select.getI());
    networkTable.getEntry("kD").setDouble(select.getD());
    networkTable.getEntry("IZone").setDouble(select.getIZone());
    networkTable.getEntry("Tolerance").setDouble(select.getErrorTolerance());
  }
}
