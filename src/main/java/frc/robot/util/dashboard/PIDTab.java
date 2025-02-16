package frc.robot.util.dashboard;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.logging.TrackedRobotPID.PIDTrack;

public class PIDTab {
  private static SendableChooser<String> pidChooser = new SendableChooser<String>();
  public static ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");
  // private static PIDController displayPID = new PIDController(0, 0, 0);
  public static String currentSelectedPID = "zEmpty";
  // ShuffleboardComponent pidWidget;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable = nt.getTable("/Shuffleboard/PID Tuning");
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double setkP = 0;
  double setkI = 0;
  double setkD = 0;

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
    currentSelectedPID = "zEmpty";
    PIDTrack.pidsTrack.put("zEmpty", RobotPIDConstants.constructPID(new PIDConstants(0)));
    pidChooser.setDefaultOption("no PID Selected", "zEmpty");
    for (String name : PIDTrack.pidsTrack.keySet()) {
      pidChooser.addOption(name, name);
    }
    pidTab.add(pidChooser).withSize(3, 2).withPosition(2, 0);
  }

  public void pidTunerBuild() {
    // pidWidget =
    //     pidTab
    //         .add("PID", displayPID)
    //         .withWidget(BuiltInWidgets.kPIDController)
    //         .withSize(2, 1)
    //         .withPosition(0, 1);

    ShuffleboardComponent kPDisplay = pidTab.addDouble("kP", () -> kP);
    ShuffleboardComponent kIDisplay = pidTab.addDouble("kI", () -> kI).withPosition(0, 1);
    ShuffleboardComponent kDDisplay = pidTab.addDouble("kD", () -> kD).withPosition(0, 2);
    GenericEntry kPSet = pidTab.add("Edit kP", kP).withPosition(0, 4).getEntry();
    GenericEntry kISet = pidTab.add("Edit kI", kI).withPosition(1, 4).getEntry();
    GenericEntry kDSet = pidTab.add("Edit kD", kD).withPosition(2, 4).getEntry();

    pidTab
        .add(
            "Set Values",
            new InstantCommand(
                () -> {
                  PIDController tempController = PIDTrack.pidsTrack.get(currentSelectedPID);
                  tempController.setP(kPSet.getDouble(-1));
                  tempController.setI(kISet.getDouble(-1));
                  tempController.setD(kDSet.getDouble(-1));
                  PIDTrack.pidsTrack.put(currentSelectedPID, tempController);
                }))
        .withPosition(3, 4);
  }

  public void periodic() {
    kP = PIDTrack.pidsTrack.get(currentSelectedPID).getP();
    kI = PIDTrack.pidsTrack.get(currentSelectedPID).getI();
    kD = PIDTrack.pidsTrack.get(currentSelectedPID).getD();
    if (checkDifferentPID()) {
      currentSelectedPID = pidChooser.getSelected();
      updateNetworkTable();
      System.out.println(currentSelectedPID);
    }
    // if (!checkDifferentPID()
    //     && ((kP != PIDTrack.pidsTrack.get(currentSelectedPID).getP())
    //         || (kI != PIDTrack.pidsTrack.get(currentSelectedPID).getI())
    //         || (kD != PIDTrack.pidsTrack.get(currentSelectedPID).getD()))) {
    //   PIDTrack.pidsTrack.get(currentSelectedPID).setP(kP);
    //   PIDTrack.pidsTrack.get(currentSelectedPID).setI(kI);
    //   PIDTrack.pidsTrack.get(currentSelectedPID).setD(kD);
    // }

    // updateNetworkTable();
  }

  public void updateNetworkTable() {
    networkTable.getEntry("Edit kP").setDouble(PIDTrack.pidsTrack.get(currentSelectedPID).getP());
    networkTable.getEntry("Edit kI").setDouble(PIDTrack.pidsTrack.get(currentSelectedPID).getI());
    networkTable.getEntry("Edit kD").setDouble(PIDTrack.pidsTrack.get(currentSelectedPID).getD());
    //   networkTable.getEntry("setpoint").setDouble(displayPID.getSetpoint());
    //   networkTable.getEntry("total error").setDouble(displayPID.getAccumulatedError());
    //   networkTable.getEntry("izone").setDouble(displayPID.getIZone());
    //   networkTable.getEntry("error").setDouble(displayPID.getError());
    //   networkTable.getEntry("error derivative").setDouble(displayPID.getErrorDerivative());
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
