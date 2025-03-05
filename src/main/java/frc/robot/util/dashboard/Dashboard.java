package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.TestConfig;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralouttake.CoralOuttakeIO;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.util.ConfigEnums.TestMode.TestingSetting;
import frc.robot.util.sysid.CreateSysidCommand;
import java.util.function.BooleanSupplier;

public class Dashboard {

  private DriveSubsystem driveSubsystem;
  private CommandXboxController controller;

  private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private LiftSubsystem liftSubsystem;
  private GantrySubsystem gantrySubsystem;
  public PIDTab pidTab = new PIDTab();
  public PPIDTab ppidTab = new PPIDTab();
  public MAXMotorTab maxMotorTab = new MAXMotorTab();
  public FLEXMotorTab flexMotorTab = new FLEXMotorTab();
  private ClimberSubsystem climberSubsystem;

  public Dashboard(
      DriveSubsystem driveSubsystem,
      LiftSubsystem liftSubsystem,
      GantrySubsystem gantrySubsystem,
      ClimberSubsystem climberSubsystem,
      CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.liftSubsystem = liftSubsystem;
    this.gantrySubsystem = gantrySubsystem;
    this.controller = controller;
    this.climberSubsystem = climberSubsystem;
    autoInit();
    teleopInit();
    if (TestConfig.tuningMode == TestingSetting.pidTuning) {
      pidTab.init();
      ppidTab.init();
    }
    if (TestConfig.tuningMode == TestingSetting.sysIDTesting) {
      mainInit();
    sysidInit();
    }
    if (TestConfig.tuningMode == TestingSetting.motorTest) {
      maxMotorTab.init();
      flexMotorTab.init();
    }
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  private void mainInit() {
    ShuffleboardTab mainTab = Shuffleboard.getTab("MAIN");
    mainTab
        .addInteger(
            "Time Left in Match:", () -> Math.round(DriverStation.getMatchTime() * 10000) / 10000)
        .withSize(5, 3)
        .withPosition(0, 0);
    mainTab.addBoolean("Coral in Bay?", () -> CoralOuttakeIO.coralDetected()).withSize(5, 3).withPosition(0, 3);
    mainTab.addBoolean("Algae in Bay?", () -> test2).withSize(5, 3).withPosition(0, 6);
    mainTab
        .addCamera("Photon Vision", "Peak Vision", "http://localhost:1182/stream.mjpg")
        // the url for this has to be changed to real photon vision when implemented
        .withSize(9, 9)
        .withPosition(5, 0);
    mainTab.addInteger("Height of Coral Getter", () -> test5).withSize(5, 3).withPosition(14, 0);
    mainTab
        .addBoolean("Robot Getting More Resources?", () -> test3)
        .withSize(5, 3)
        .withPosition(14, 3);
    mainTab
        .addBoolean("This is here to make all this look nice.", () -> true)
        .withSize(5, 3)
        .withPosition(14, 6);
  }

  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  private void teleopInit() {
    ShuffleboardTab teleopTab = Shuffleboard.getTab("TELEOP");
    // TODO add actual url
    // steps:
    // 1. https://www.linkedin.com/pulse/howtousetheusbcameraontheorangepizero2-%E9%9B%AA-%E9%99%88
    teleopTab
        .addCamera("Rear Cam", "BackLL", "http://photonvision.local:5800")
        .withSize(5, 4)
        .withPosition(1, 1);
    teleopTab
        .addBoolean("Left Sensor", () -> climberSubsystem.getSensor1())
        .withSize(1, 1)
        .withPosition(1, 0);
    teleopTab
        .addBoolean("Right Sensor", () -> climberSubsystem.getSensor2())
        .withSize(1, 1)
        .withPosition(5, 0);
  }

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
}
