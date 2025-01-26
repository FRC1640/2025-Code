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
    super.periodic();
  }

  public double getleaderMotorPosition() {
    return inputs.leadermotorPosition;
  }

  public double getfollowerMotorPosition() {
    return inputs.followermotorPosition;
  }

  public double getleaderMotorVelocity() {
    return inputs.leadermotorVelocity;
  }

  public double getfollowerMotorVelocity() {
    return inputs.followermotorVelocity;
  }

  public double getleaderMotorCurrent() {
    return inputs.leadermotorCurrent;
  }

  public double getfollowerMotorCurrent() {
    return inputs.followermotorCurrent;
  }

  public double getleaderMotorVoltage() {
    return inputs.leadermotorVoltage;
  }
}
