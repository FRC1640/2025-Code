package frc.robot.subsystems.gantry.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;

public class MoveToPositionCommand {
  private static SparkMax gantryMax;
  private static RelativeEncoder encoder;
  private static PIDController gantryPID;

  public MoveToPositionCommand(SparkMax gantrySparkMax, RelativeEncoder encoderx) {
    this.gantryMax = gantrySparkMax;
    this.encoder = encoderx;
    this.gantryPID = RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID);
  }

  public static Command moveToPositionCommand(int pos) {
    Command c =
        new Command() {
          @Override
          public void execute() {
            setPos(pos);
          }
        };
    return c;
  }

  private static void setPos(double pos) {
    gantryMax.set(gantryPID.calculate(encoder.getPosition(), pos));
  }

  // private static double metersToRotations(double meterPos) {
  //   return (2 * meterPos * Math.PI)
  //       / (GantryConstants.pulleyRadiusIn
  //           * .0254); // hanging out here for now in case this comes in handy
  // }
}
