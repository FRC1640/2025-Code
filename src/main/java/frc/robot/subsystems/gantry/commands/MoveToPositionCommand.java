package frc.robot.subsystems.gantry.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
// import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPositionCommand {
  private static SparkMax gantryMax;
  private static RelativeEncoder encoder;
  private static PIDController gantryPID;

  public MoveToPositionCommand(SparkMax gantrySparkMax, RelativeEncoder encoderx) {
    this.gantryMax = gantrySparkMax;
    this.encoder = encoderx;
    this.gantryPID = RobotPIDConstants.constructPID(RobotPIDConstants.gantryPid);
  }

  // public static Command moveToPositionCommand(int pos) {
  //    Command c =
  //        new Command() {
  //          @Override
  //          public void execute() {
  //            switch (pos) {
  //              case 0: // left
  //                toPos(0);
  //                break;
  //              case 1:
  //                // center
  //                break;
  //              case 2:
  //                // right
  //                break;
  //            }
  //          }
  //        };
  //    return c;
  // }

  private static void setPos(double pos) {
    gantryMax.set(gantryPID.calculate(encoder.getPosition(), pos));
  }
}
