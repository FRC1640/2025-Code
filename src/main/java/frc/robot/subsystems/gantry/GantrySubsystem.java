package frc.robot.subsystems.gantry;

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
}
