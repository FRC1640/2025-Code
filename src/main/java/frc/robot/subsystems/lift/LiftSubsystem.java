package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LiftSubsystem extends SubsystemBase {
  LiftIO liftIO;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();

  public LiftSubsystem(LiftIO liftIO) {
    this.liftIO = liftIO;
  }

  @Override
  public void periodic() {

    liftIO.updateInputs(inputs);
    Logger.processInputs("Lift/", inputs);
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

  public double getLeaderTemperature() {
    return inputs.leaderTemperature;
  }

  public double getFollowerTemperature() {
    return inputs.followerTemperature;
  }

  public void setLiftPosition(double pos) {
    liftIO.setLiftPosition(pos, liftIO.getInputs());
  }

  public void setLiftVoltage(double voltage) {
    liftIO.setLiftVoltage(voltage, liftIO.getInputs());
  }
}
