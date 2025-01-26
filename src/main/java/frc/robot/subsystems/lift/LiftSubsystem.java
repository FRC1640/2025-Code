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

  public double getMotor1Position() {
    return inputs.leadermotorPosition;
  }

  public double getMotor2Position() {
    return inputs.followermotorPosition;
  }

  public double getMotor1Velocity() {
    return inputs.leadermotorVelocity;
  }

  public double getMotor2Velocity() {
    return inputs.followermotorVelocity;
  }

  public double getMotor1Current() {
    return inputs.leadermotorCurrent;
  }

  public double getMotor2Current() {
    return inputs.followermotorCurrent;
  }

  public double getMotor1Voltage() {
    return inputs.leadermotorVoltage;
  }
}
