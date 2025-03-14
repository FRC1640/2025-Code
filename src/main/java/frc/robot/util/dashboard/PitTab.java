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
        .withSize(2, 2)
        .withPosition(7, 2);
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
        .addString("Control Scheme", () -> "LStick Drive; RStick Climber Lift; POV Up/Down Winch")
        .withSize(4, 1)
        .withPosition(0, 0);
    ShuffleboardLayout instructLayout =
        pitTab
            .getLayout("Pit Control Bindings", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "HIDDEN"))
            .withPosition(0, 1);
    instructLayout.add("Instruct1", "L Stick Drive;");
    instructLayout.add("Instruct2", "R Stick Climber Lift");
    instructLayout.add("Instruct3", "POV Up/Down Winch");
    instructLayout.add("Instruct4", "L/R Bumpers Gantry");

  }
}
