package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.dashboard.PIDInfo.PIDCommandRegistry;
import frc.robot.util.logging.PIDTracking.ProfiledPIDTrack;

public class PPIDTab {
  private static SendableChooser<ProfiledPIDController> pidChooser =
      new SendableChooser<ProfiledPIDController>();
  public static ShuffleboardTab pidTab = Shuffleboard.getTab("ProfiledPID Tuning");
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable = nt.getTable("/Shuffleboard/ProfiledPID Tuning");

  GenericEntry kPSet;
  GenericEntry kISet;
  GenericEntry kDSet;
  GenericEntry setpoint;
  GenericEntry maxAcceleration;
  GenericEntry maxVelocity;
  GenericEntry iZoneSet;
  GenericEntry pTolerance;
  GenericEntry vTolerance;

  public PPIDTab() {}

  public void init() {
    pidTunerBuild();
  }

  public void pidTunerBuild() {
    pidChooser.setDefaultOption("none", new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));
    for (String info : ProfiledPIDTrack.pidsTrack.keySet()) {
      pidChooser.addOption(info, ProfiledPIDTrack.pidsTrack.get(info));
    }
    pidChooser.onChange((x) -> updateNetworkTable(x));
    pidTab.add(pidChooser).withSize(3, 1).withPosition(0, 0);

    kPSet = pidTab.add("kP", 0).withPosition(0, 1).getEntry();
    kISet = pidTab.add("kI", 0).withPosition(1, 1).getEntry();
    kDSet = pidTab.add("kD", 0).withPosition(2, 1).getEntry();

    maxAcceleration = pidTab.add("maxAcceleration", 0).withPosition(3, 1).getEntry();
    maxVelocity = pidTab.add("maxVelocity", 0).withPosition(4, 1).getEntry();
    setpoint = pidTab.add("Set", 0).withPosition(0, 2).withSize(1, 1).getEntry();
    iZoneSet = pidTab.add("IZone", 0).withPosition(5, 1).getEntry();
    pTolerance = pidTab.add("Pos Tolerance", 0).withPosition(6, 1).getEntry();
    vTolerance = pidTab.add("Vel Tolerance", 0).withPosition(6, 1).getEntry();

    pidTab
        .add(
            "Run Attached PID Command",
            new InstantCommand(
                () -> {
                  if (PIDCommandRegistry.profiledAttachedCommands.get(pidChooser.getSelected())
                      != null) {
                    if (PIDCommandRegistry.currentlyRunningCommand != null) {
                      CommandScheduler.getInstance()
                          .cancel(PIDCommandRegistry.currentlyRunningCommand);
                    }

                    PIDCommandRegistry.currentlyRunningCommand =
                        PIDCommandRegistry.profiledAttachedCommands
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
                  pidChooser.getSelected().setP(kPSet.getDouble(0));
                  pidChooser.getSelected().setI(kISet.getDouble(0));
                  pidChooser.getSelected().setD(kDSet.getDouble(0));
                  pidChooser.getSelected().setIZone(iZoneSet.getDouble(0));
                  pidChooser
                      .getSelected()
                      .setTolerance(pTolerance.getDouble(0), vTolerance.getDouble(0));
                  pidChooser
                      .getSelected()
                      .setConstraints(
                          new Constraints(maxVelocity.getDouble(0), maxAcceleration.getDouble(0)));
                }))
        .withPosition(7, 1);
  }

  public void updateNetworkTable(ProfiledPIDController select) {
    networkTable.getEntry("kP").setDouble(select.getP());
    networkTable.getEntry("kI").setDouble(select.getI());
    networkTable.getEntry("kD").setDouble(select.getD());
    networkTable.getEntry("maxAcceleration").setDouble(select.getConstraints().maxAcceleration);
    networkTable.getEntry("maxVelocity").setDouble(select.getConstraints().maxVelocity);
    networkTable.getEntry("IZone").setDouble(select.getIZone());
    networkTable.getEntry("Pos Tolerance").setDouble(select.getPositionTolerance());
    networkTable.getEntry("Vel Tolerance").setDouble(select.getVelocityTolerance());
  }
}
