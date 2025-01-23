package frc.robot.subsystems.gantry.commands;

import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPositionCommand {
  private static SparkMax gantryMax;

  public MoveToPositionCommand(SparkMax gantrySparkmax) {
    this.gantryMax = gantrySparkmax;
  }

  // public static Command moveToPositionCommand(int pos) {
  //   Command c =
  //       new Command() {
  //         @Override
  //         public void execute() {
  //           switch (pos) {
  //             case 0: // left
  //               toPos(0);
  //               break;
  //             case 1:
  //               // center
  //               break;
  //             case 2:
  //               // right
  //               break;
  //           }
  //         }
  //       };
  //   return c;
  // }

  // private static void toPos(double pos) {
  //   gantryMax.setPosition(pos);
  // }
}
