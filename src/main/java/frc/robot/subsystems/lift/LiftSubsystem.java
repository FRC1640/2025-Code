package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  LiftIOSpark liftIO;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();

  public LiftSubsystem(LiftIOSpark liftIO) {
    this.liftIO = liftIO;
  }

  @Override
  public void periodic() {
    liftIO.updateInputs(inputs);
  }

  public double getLeaderMotorPosition() {
    return inputs.leaderMotorPosition;
  }

  public double getFollowerMotorPosition() {
    return inputs.followerMotorPosition;
  }

  public double getLeaderMotorVelocity() {
    return inputs.leaderMotorVelocity;
  }

  public double getFollowerMotorVelocity() {
    return inputs.followerMotorVelocity;
  }

  public double getLeaderMotorCurrent() {
    return inputs.leaderMotorCurrent;
  }

  public double getFollowerMotorCurrent() {
    return inputs.followerMotorCurrent;
  }

  public double getLeaderMotorVoltage() {
    return inputs.leaderMotorVoltage;
  }
}
