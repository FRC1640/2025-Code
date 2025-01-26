package frc.robot.subsystems.lift;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;

public class LiftIOSim implements LiftIO {
  private final DCMotorSim motor1Sim;
  private final DCMotorSim motor2Sim;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();

  public LiftIOSim() {
    DCMotor motor1SimGearbox = DCMotor.getNEO(1);
    DCMotor motor2SimGearbox = DCMotor.getNEO(1);

    motor1Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor1SimGearbox, 0.00019125, RobotConstants.LiftConstants.motorRatio),
            motor1SimGearbox);
    motor2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor2SimGearbox, 0.00019125, RobotConstants.LiftConstants.motorRatio),
            motor2SimGearbox);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.liftmotor1Voltage = motor1Sim.getInputVoltage();
    inputs.liftmotor2Voltage = motor2Sim.getInputVoltage();
    inputs.liftmotor1Current = motor1Sim.getCurrentDrawAmps();
    inputs.liftmotor2Current = motor2Sim.getCurrentDrawAmps();
    inputs.liftmotor1Position = motor1Sim.getAngularVelocityRPM();
    inputs.liftmotor2Position = motor2Sim.getAngularVelocityRPM();
    inputs.liftmotor1Velocity = motor1Sim.getAngularVelocityRPM();
    inputs.liftmotor2Velocity = motor2Sim.getAngularVelocityRPM();
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
