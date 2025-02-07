package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.util.dashboard.PIDMap.PIDKey;
import frc.robot.util.sysid.CreateSysidCommand;
import java.util.HashMap;
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

  private boolean pid = false;
  private boolean sysid = false;

  public Dashboard(
      DriveSubsystem driveSubsystem,
      LiftSubsystem liftSubsystem,
      CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.liftSubsystem = liftSubsystem;
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
    testTab.add("TEST?", testChooser).withSize(4, 3).withPosition(4, 1);
    // System.out.println(testChooser.getSelected() + "sanity check");
    testChooser.onChange((x) -> testChange(x, PIDMap.pidMap));
  }

  private void testChange(Test test, HashMap<PIDKey, PIDController> pidKey) {
    switch (test) { // come back to allow multiple test tabs happening
      case SYSID:
        if (!sysid) {
          pid = false;
          sysid = true;
          sysidInit();
        }
        break;
      case PID:
        if (!pid) {
          pid = true;
          sysid = false;
          pidInit(pidKey);
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
    ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
    sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
  }

  public Command getSysidCommand() {
    return sysidChooser.getSelected();
  }

  private PIDController pidSelected;

  private void pidInit(HashMap<PIDKey, PIDController> pidKeys) {
    SendableChooser<String> pidChooser = new SendableChooser<String>();
    pidChooser.setDefaultOption("none :/", "defaultPID");
    // for (int i = 0; i <= PIDMap.pidMap.size(); i++) {
    //   pidChooser.addOption(PIDMap.pidMap.toString(), PIDMap.pidMap.get(PIDKey.GANTRY));
    // } // might just do this manually rather than for loop tbd

    ShuffleboardTab pidTab = Shuffleboard.getTab("PID");
    pidTab.add("PID?", pidChooser).withPosition(0, 0).withSize(3, 1);
    pidTab.add("P", pidSelected.getP()).withPosition(1, 0).withSize(1, 1);
    pidTab.add("I", pidSelected.getI()).withPosition(1, 1).withSize(1, 1);
    pidTab.add("D", pidSelected.getD()).withPosition(1, 2).withSize(1, 1);
  }
  ;

  // private void pidChange() {
  //   pid
  // }
}
