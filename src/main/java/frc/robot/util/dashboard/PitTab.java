package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import java.util.Map;

public class PitTab {
  public static ShuffleboardTab pitTab;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable;
  private GantrySubsystem gantrySubsystem;
  private LiftSubsystem liftSubsystem;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private AlgaeSubsystem algaeIntakeSubsystem;
  private WinchSubsystem winchSubsystem;

  public PitTab(
      GantrySubsystem gantrySubsystem,
      LiftSubsystem liftSubsystem,
      CoralOuttakeSubsystem coralOuttakeSubsystem,
      ClimberSubsystem climberSubsystem,
      AlgaeSubsystem algaeIntakeSubsystem,
      WinchSubsystem winchSubsystem) {
    this.gantrySubsystem = gantrySubsystem;
    this.liftSubsystem = liftSubsystem;
    this.coralOuttakeSubsystem = coralOuttakeSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    this.winchSubsystem = winchSubsystem;
  }

  public void init() {
    pitTab = Shuffleboard.getTab("Pit");
    networkTable = nt.getTable("/Shuffleboard/Pit Testing");
    pitTesterBuild();
  }

  public void pitTesterBuild() {
    pitTab
        .addCamera("BackCamera", "USB Camera 0", "http://10.16.40.52:1186/stream.mjpg")
        .withSize(4, 3)
        .withPosition(5, 2);
    pitTab
        .addBoolean("Left Sensor", () -> climberSubsystem.getSensor1())
        .withSize(1, 1)
        .withPosition(7, 0);
    pitTab
        .addBoolean("Right Sensor", () -> climberSubsystem.getSensor2())
        .withSize(1, 1)
        .withPosition(8, 0);
    pitTab
        .addBoolean("Has Algae?", () -> algaeIntakeSubsystem.hasAlgae())
        .withSize(1, 1)
        .withPosition(7, 1);
    pitTab
        .addBoolean("Has Coral?", () -> coralOuttakeSubsystem.hasCoral())
        .withSize(1, 1)
        .withPosition(8, 1);
    pitTab
        .addString(
            "PIT Controller", () -> "Switch your controller to Port 4 and Preset Board on Port 2")
        .withSize(5, 1)
        .withPosition(0, 0);
    pitTab
        .addString(
            "IMPORTANT", () -> "To Make This Work, Make sure to be in TEST Mode, not Teleoperated")
        .withSize(5, 1)
        .withPosition(0, 4);
    ShuffleboardLayout instructLayout =
        pitTab
            .getLayout("Pit Control Bindings", BuiltInLayouts.kList)
            .withSize(3, 3)
            .withProperties(Map.of("Label position", "HIDDEN"))
            .withPosition(0, 1);
    instructLayout.add("Instruct1", "L Stick: Drive");
    instructLayout.add("Instruct2", "R Stick: Climber Lift");
    instructLayout.add("Instruct3", "POV Up/Down: Winch");
    instructLayout.add("Instruct4", "POV Right: Clamp Climber");
    instructLayout.add("Instruct5", "Start: Zero Lift Encoders");
    instructLayout.add("Instruct6", "& Start: Enable Climber PID");
    instructLayout.add("Instruct7", "L/R Bumpers: Gantry");
    instructLayout.add("Instruct8", "Back Button: Gantry Home");
    instructLayout.add("Instruct9", "Right Trigger: Algae Intake");
    instructLayout.add("Instruct10", "Left Trigger: Algae Outtake");
    instructLayout.add("Instruct11", "POV Left: Disable Climber PID");
    instructLayout.add("Instruct12", "X: Disable Antitip");

    ShuffleboardLayout instructLayoutCoral =
        pitTab
            .getLayout("IMPORTANT BINDINGS", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withProperties(Map.of("Label position", "HIDDEN"))
            .withPosition(3, 1);
    instructLayoutCoral.add("Instruct1", "Start: Zero Lift Encoders");
    instructLayoutCoral.add("Instruct2", "& Start: Enable Climber PID");
    instructLayoutCoral.add("Instruct3", "Use Preset board for Lift");
    instructLayoutCoral.add("Instruct4", "B: Outtake Coral");
    instructLayoutCoral.add("Instruct5", "Y: Safe pos");
    instructLayoutCoral.add("Instruct6", "A: Confirm preset");
    instructLayoutCoral.add("Instruct7", "X: Disable Antitip");

    ShuffleboardLayout canID =
        pitTab.getLayout("Can IDs", BuiltInLayouts.kList).withSize(2, 2).withPosition(5, 0);
    canID.add("Front Left Drive", "1");
    canID.add("Front Left Steering", "2");
    canID.add("Front Right Drive", "3");
    canID.add("Front Right Steering", "4");
    canID.add("Back Right Drive", "5");
    canID.add("Back Right Steering", "6");
    canID.add("Back Left Drive", "7");
    canID.add("Back Left Steering", "8");
    canID.add("Elevator-Master", "9");
    canID.add("Elevator-Slave", "10");
    canID.add("Algae Intake-Left", "11");
    canID.add("Algae Intake-Right", "12");
    canID.add("Climber-Angle Master", "13");
    canID.add("Climber-Angle Slave", "14");
    canID.add("Climber-Up-Down", "15");
    canID.add("Coral Outtake", "16");
    canID.add("Coral Gantry", "17");
    canID.add("LaserCAN", "18");
    canID.add("Power Dist Hub", "21");
    canID.add("Pneumatics Hub", "22");
  }
}
