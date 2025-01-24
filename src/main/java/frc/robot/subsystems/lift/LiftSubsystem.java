package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  LiftIO io;
  // PIDController liftPID = RobotPIDConstants.constructPID(RobotPIDConstants.liftPID); pid

  public LiftSubsystem(LiftIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command liftPIDCommand(int pos) {
    Command c =
        new Command() {
          @Override
          public void execute() {
            setLiftPosition(pos);
          }
        };
    return c;
  }

  public void setLiftPosition(int pos) {
    switch (pos) {
      case 0:
        // bottom
        break;
      case 1:
        // middle
        break;
      case 2:
        // top
        break;
    }
  }
}
