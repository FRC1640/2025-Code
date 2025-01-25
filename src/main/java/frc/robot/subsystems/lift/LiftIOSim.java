package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class LiftIOSim implements LiftIO {
  private final DCMotorSim motor1Sim;
  private final DCMotorSim motor2Sim;

  public LiftIOSim() {
    motor1Sim = new DCMotorSim();
    motor2Sim = new DCMotorSim();
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {}
}
