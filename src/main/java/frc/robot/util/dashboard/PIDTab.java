package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.util.tools.logging.TrackedRobotPID.PIDTrack;

public class PIDTab {
  private static SendableChooser<String> pidChooser = new SendableChooser<String>();
  public static ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");
  private static PIDController displayPID = new PIDController(0, 0, 0);
  public static String currentSelectedPID = "";
  ShuffleboardComponent pidWidget;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable = nt.getTable("/Shuffleboard/PID Tuning/PID");

  public PIDTab() {}

  public void init() {
    widgetBuild();
  }

  public void widgetBuild() {
    selectorBuild();
    pidTunerBuild();
    buttonBuild();
  }

  public void buttonBuild() {}

  public void selectorBuild() {
    PIDTrack.pidsTrack.put("empty", displayPID);
    pidChooser.setDefaultOption("no PID Selected", "empty");
    for (String name : PIDTrack.pidsTrack.keySet()) {
      pidChooser.addOption(name, name);
    }
    pidTab.add(pidChooser).withSize(4, 3).withPosition(3, 0);
  }

  public void pidTunerBuild() {
    pidWidget =
        pidTab
            .add("PID", displayPID)
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 1)
            .withPosition(0, 1);
  }

  public void periodic() {

    if (checkDifferentPID() == false
        && displayPID != PIDTrack.pidsTrack.get(pidChooser.getSelected())) {
      System.out.println("cool");
      PIDTrack.pidsTrack.put(currentSelectedPID, displayPID);
    }
    if (checkDifferentPID()) {
      displayPID = PIDTrack.pidsTrack.get(pidChooser.getSelected());
      currentSelectedPID = pidChooser.getSelected();
    }
    updateNetworkTable();
    // System.out.println(currentSelectedPID);
  }

  public void updateNetworkTable() {
    networkTable.getEntry("p").setDouble(displayPID.getP());
    networkTable.getEntry("i").setDouble(displayPID.getI());
    networkTable.getEntry("d").setDouble(displayPID.getD());
    networkTable.getEntry("setpoint").setDouble(displayPID.getSetpoint());
    networkTable.getEntry("total error").setDouble(displayPID.getAccumulatedError());
    networkTable.getEntry("izone").setDouble(displayPID.getIZone());
    networkTable.getEntry("error").setDouble(displayPID.getError());
    networkTable.getEntry("error derivative").setDouble(displayPID.getErrorDerivative());
  }

  public boolean checkDifferentPID() {
    if (currentSelectedPID != pidChooser.getSelected()) {
      return true;
    }
    return false;
  }

  // public void pidInit() {
  //
  //   ShuffleboardComponent errorDerivativeWidget =
  //       pidTab
  //           .addDouble(
  //               "Error Derivative",
  //               () -> PIDTrack.pidsTrack.get(pidChooser.getSelected()).getErrorDerivative())
  //           .withSize(1, 1)
  //           .withPosition(6, 0);
  //   ShuffleboardComponent errorWidget =
  //       pidTab
  //           .addDouble("Error", () ->
  // PIDTrack.pidsTrack.get(pidChooser.getSelected()).getError())
  //           .withSize(1, 1)
  //           .withPosition(7, 0);
  //   ShuffleboardComponent setpointWidget =
  //       pidTab
  //           .addDouble(
  //               "Setpoint", () -> PIDTrack.pidsTrack.get(pidChooser.getSelected()).getSetpoint())
  //           .withSize(1, 1)
  //           .withPosition(8, 0);
  //   ShuffleboardComponent atSetpoint =
  //       pidTab
  //           .addBoolean(
  //               "At Setpoint", () ->
  // PIDTrack.pidsTrack.get(pidChooser.getSelected()).atSetpoint())
  //           .withSize(1, 1)
  //           .withPosition(9, 0);
  //   // Tune Values
  //   ShuffleboardComponent setkPSlider =
  //       pidTab
  //           .add("Set kP", PIDTrack.pidsTrack.get(pidChooser.getSelected()))
  //           .withWidget(BuiltInWidgets.kPIDController)
  //           .withSize(2, 1)
  //           .withPosition(0, 1);
  //   pidTab
  //       .add(
  //           "Apply PID Changes",
  //           new InstantCommand(
  //               () -> {
  //                 PIDController constructedPidController =
  //                     PIDTrack.pidsTrack.get(pidChooser.getSelected());
  //                 // constructedPidController.setP(kPSupplier.getAsDouble());
  //                 // PIDTrack.pidsTrack.put(pidChooser.getSelected(), constructedPidController);
  //               }))
  //       .withPosition(0, 3)
  //       .withSize(2, 1);
  // }
}
