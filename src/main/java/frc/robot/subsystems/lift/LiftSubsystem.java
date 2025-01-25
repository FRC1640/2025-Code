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
    return inputs.liftmotor1Position;
  }

  public double getMotor2Position() {
    return inputs.liftmotor2Position;
  }

  public double getMotor1Velocity() {
    return inputs.liftmotor1Velocity;
  }

  public double getMotor2Velocity() {
    return inputs.liftmotor2Velocity;
  }

  public double getMotor1Current() {
    return inputs.liftmotor1Current;
  }

  public double getMotor2Current() {
    return inputs.liftmotor2Current;
  }

  public double getMotor1Voltage() {
    return inputs.liftmotor1Voltage;
  }
}
