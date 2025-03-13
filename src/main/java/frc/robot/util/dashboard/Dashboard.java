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
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.util.ConfigEnums.TestMode.TestingSetting;
import frc.robot.util.sysid.CreateSysidCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
  public PitTab pitTab;

  private ClimberSubsystem climberSubsystem;
  private AlgaeSubsystem algaeSubsystem;
  private CoralOuttakeSubsystem coralSubsystem;

  public Dashboard(
      DriveSubsystem driveSubsystem,
      LiftSubsystem liftSubsystem,
      GantrySubsystem gantrySubsystem,
      ClimberSubsystem climberSubsystem,
      AlgaeSubsystem algaeSubsystem,
      CoralOuttakeSubsystem coralSubsystem,
      WinchSubsystem winchSubsystem,
      CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.liftSubsystem = liftSubsystem;
    this.gantrySubsystem = gantrySubsystem;
    this.controller = controller;
    this.climberSubsystem = climberSubsystem;
    this.algaeSubsystem = algaeSubsystem;
    this.coralSubsystem = coralSubsystem;
    autoInit();
    teleopInit();
    if (TestConfig.tuningMode == TestingSetting.pidTuning) {
      pidTab.init();
      ppidTab.init();
    }
    if (TestConfig.tuningMode == TestingSetting.sysIDTesting) {
      sysidInit();
    }
    if (TestConfig.tuningMode == TestingSetting.motorTest) {
      maxMotorTab.init();
      flexMotorTab.init();
    }
    if (TestConfig.tuningMode == TestingSetting.pit) {
      pitTab = new PitTab(gantrySubsystem, liftSubsystem, coralSubsystem, climberSubsystem, algaeSubsystem, winchSubsystem);
      pitTab.init();
    }
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  DoubleSupplier time = () -> (Math.round(DriverStation.getMatchTime() * 10000) / 10000);

  private void teleopInit() {
    ShuffleboardTab teleopTab = Shuffleboard.getTab("TELEOP");
    // HttpCamera httpCamera = new HttpCamera("FrontCamera", "http://10.16.40.52:1182/stream.mjpg");
    // httpCamera.setResolution(1280 / 5, 720 / 5);
    // httpCamera.setFPS(15);
    // teleopTab.add("Front Cam", httpCamera).withSize(3, 3).withPosition(2, 1);
    // UsbCamera camera = new UsbCamera("BackCamera", 1);
    // camera.setFPS(30);
    // camera.setResolution(1920 / 2, 1080 / 2);
    // teleopTab.add(camera);

    teleopTab
        .addCamera("BackCamera", "USB Camera 0", "http://10.16.40.52:1186/stream.mjpg")
        .withSize(4, 3)
        .withPosition(5, 1);
    teleopTab
        .addBoolean("Left Sensor", () -> climberSubsystem.getSensor1())
        .withSize(2, 1)
        .withPosition(5, 0);
    teleopTab
        .addBoolean("Right Sensor", () -> climberSubsystem.getSensor2())
        .withSize(2, 1)
        .withPosition(7, 0);
    teleopTab.addDouble("Match Timer", time).withSize(3, 1).withPosition(2, 0);
    teleopTab
        .addBoolean("Has Algae?", () -> algaeSubsystem.hasAlgae())
        .withSize(1, 4)
        .withPosition(0, 0);
    teleopTab
        .addBoolean("Has Coral?", () -> coralSubsystem.hasCoral())
        .withSize(1, 4)
        .withPosition(1, 0);
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
