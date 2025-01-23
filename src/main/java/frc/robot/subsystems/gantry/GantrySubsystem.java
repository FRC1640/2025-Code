package frc.robot.subsystems.gantry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GantrySubsystem extends SubsystemBase {
  GantryIOInputsAutoLogged inputs = new GantryIOInputsAutoLogged();
  GantryIO io;
  // PIDController gantryPID = RobotPIDConstants.constructPID(RobotPIDConstants.carriagePID); to be
  // explored for more accurate alignment

  public GantrySubsystem(GantryIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command gantryPIDCommand(int pos) {
    Command c =
        new Command() {
          @Override
          public void execute() {
            setCarriagePosition(pos);
          }
        };
    return c;
  }

  public void setCarriagePosition(int pos) {
    switch (pos) {
      case 0:
        // left
        break;
      case 1:
        // center
        break;
      case 2:
        // right
        break;
    }
  }
}
