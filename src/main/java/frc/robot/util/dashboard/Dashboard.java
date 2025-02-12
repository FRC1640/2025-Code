package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
// import frc.robot.util.dashboard.PIDMap.PIDKey;
import frc.robot.util.sysid.CreateSysidCommand;
// import java.util.Map;
// import java.util.Map.Entry;
import java.util.function.BooleanSupplier;

public class Dashboard {

  public enum Test {
    PID,
    SYSID,
    NONE
  };

  private DriveSubsystem driveSubsystem;
  private CommandXboxController controller;

  private SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private LiftSubsystem liftSubsystem;
  private GantrySubsystem gantrySubsystem;

  private boolean pid = false;
  private boolean sysid = false;

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
    testInit();
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  private void teleopInit() {}

  private void testInit() {
    SendableChooser<Test> testChooser = new SendableChooser<Test>();
    testChooser.setDefaultOption("womp womp", Test.NONE);
    testChooser.addOption("PID", Test.PID);
    testChooser.addOption("SYSID", Test.SYSID);
    ShuffleboardTab testTab = Shuffleboard.getTab("TEST");
    testTab.add("TEST?", testChooser).withSize(4, 3).withPosition(1, 1);
    testChooser.onChange((x) -> testChange(x));
  }

  private void testChange(Test test) {
    switch (test) {
      case SYSID:
        if (!sysid) {
          sysidInit();
        }
        break;
      case PID:
        if (!pid) {
          pidInit();
        }
        break;
      case NONE:
        break;
    }
  }

  private void sysidInit() {
    pid = false;
    sysid = true;
    BooleanSupplier startNext = controller.b();
    BooleanSupplier cancel = controller.a();
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
    ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
    sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
  }

  public Command getSysidCommand() {
    return sysidChooser.getSelected();
  }

  private static GenericEntry kP;
  private static GenericEntry kI;
  private static GenericEntry kD;
  private static GenericEntry setpt;

  private void pidInit() {
    pid = true;
    sysid = false;

    SendableChooser<PIDController> pidChooser = new SendableChooser<PIDController>();
    pidChooser.setDefaultOption("none :/", new PIDController(0, 0, 0));
    for (String controllerName : PIDMap.pidMap.keySet()) {
      pidChooser.addOption(controllerName, PIDMap.pidMap.get(controllerName));
    }

    PIDMap.setEntries(kP, kI, kD, setpt);

    ShuffleboardTab pidTab = Shuffleboard.getTab("PID");
    pidTab.add("PID?", pidChooser).withPosition(0, 0).withSize(3, 1);
    kP = pidTab.add("P", 0).withPosition(0, 1).withSize(1, 1).getEntry();
    kI = pidTab.add("I", 0).withPosition(1, 1).withSize(1, 1).getEntry();
    kD = pidTab.add("D", 0).withPosition(2, 1).withSize(1, 1).getEntry();
    setpt = pidTab.add("SETPOINT", 0).withPosition(1, 2).withSize(1, 1).getEntry();

    pidChooser.onChange((x) -> pidChange(x));
  }
  ;

  private void pidChange(PIDController pid) {
    PIDMap.setPID(pid);
    pid.setP(kP.getDouble(0));
    pid.setI(kI.getDouble(0));
    pid.setD(kD.getDouble(0));
    pid.setSetpoint(setpt.getDouble(0));
  }
}
