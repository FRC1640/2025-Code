package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.util.sysid.CreateSysidCommand;
import frc.robot.util.tools.logging.TrackedRobotPID.PIDTrack;
import java.util.function.BooleanSupplier;

public class Dashboard {

  private DriveSubsystem driveSubsystem;
  private CommandXboxController controller;

  private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private static SendableChooser<String> pidChooser = new SendableChooser<String>();

  private LiftSubsystem liftSubsystem;
  private GantrySubsystem gantrySubsystem;
  private static String lastPID = "";
  public static String lastPIDName = "";
  public static ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");

  public Dashboard(
      DriveSubsystem driveSubsystem,
      LiftSubsystem liftSubsystem,
      GantrySubsystem gantrySubsystem,
      CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.liftSubsystem = liftSubsystem;
    this.gantrySubsystem = gantrySubsystem;
    this.controller = controller;
    autoInit();
    teleopInit();
    sysidInit();
    pidInit();
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  public void pidInit() {
    PIDTrack.pidsTrack.put("zEmpty", new PIDController(-1, -1, -1));
    pidChooser.setDefaultOption("no PID Selected", "zEmpty");
    for (String name : PIDTrack.pidsTrack.keySet()) {
      pidChooser.addOption(name, name);
    }

    pidTab.add(pidChooser).withSize(3, 1).withPosition(0, 0);
    ShuffleboardComponent kPtab =
        Dashboard.pidTab
            .addDouble("kP", () -> PIDTrack.pidsTrack.get(pidChooser.getSelected()).getP())
            .withSize(1, 1)
            .withPosition(3, 0);
    ShuffleboardComponent kItab =
        Dashboard.pidTab
            .addDouble("kI", () -> PIDTrack.pidsTrack.get(pidChooser.getSelected()).getI())
            .withSize(1, 1)
            .withPosition(4, 0);
    ShuffleboardComponent kPTab =
            Dashboard.pidTab
                .addDouble("kP", () -> PIDTrack.pidsTrack.get(pidChooser.getSelected()).getP())
                .withSize(1, 1)
                .withPosition(5, 0); 
  }

  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  private void teleopInit() {}

  private void sysidInit() {
    BooleanSupplier startNext = controller.b();
    BooleanSupplier cancel = controller.a();
    ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
    sysidChooser.setDefaultOption("none :/", new WaitCommand(0.1));
    sysidChooser.addOption(
        "Swerve",
        CreateSysidCommand.createCommand(
            driveSubsystem::sysIdQuasistatic,
            driveSubsystem::sysIdDynamic,
            "Swerve",
            startNext,
            cancel,
            () -> driveSubsystem.stop()));
    sysidChooser.addOption(
        "Lift",
        CreateSysidCommand.createCommand(
            liftSubsystem::sysIdQuasistatic,
            liftSubsystem::sysIdDynamic,
            "Lift",
            startNext,
            cancel,
            () -> liftSubsystem.setLiftVoltage(0)));

    sysidChooser.addOption(
        "Gantry",
        CreateSysidCommand.createCommand(
            gantrySubsystem::sysIdQuasistatic,
            gantrySubsystem::sysIdDynamic,
            "Gantry",
            startNext,
            cancel,
            () -> gantrySubsystem.setGantryVoltage(0)));
    sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
  }

  public static Command getSysidCommand() {
    return sysidChooser.getSelected();
  }

  public static String getSelectedPID() {
    return pidChooser.getSelected();
  }

  public static boolean isDifferentTab() {
    if (pidChooser.getSelected() == lastPID) {
      lastPID = pidChooser.getSelected();

      return false;
    } else {
      lastPID = pidChooser.getSelected();

      return true;
    }
  }
}
