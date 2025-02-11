package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim liftSim;
  private final DoubleSolenoidSim doubleSolenoidSim;
  private final DigitalInput sensor1Sim, sensor2Sim;
  private final PIDController liftPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);

  public ClimberIOSim() {
    DCMotor motor1SimGearbox = DCMotor.getNEO(1);

    liftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor1SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor1SimGearbox);
    doubleSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);
    sensor1Sim = new DigitalInput(ClimberConstants.sensor1Channel);
    sensor2Sim = new DigitalInput(ClimberConstants.sensor2Channel);
  }

  @Override
  public void setClimberLiftVoltage(double voltage, ClimberIOInputs inputs) {
    liftSim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(inputs.liftMotorPosition, voltage, ClimberConstants.liftLimits)));
  }

  @Override
  public void setClimberLiftPosition(double position, ClimberIOInputs inputs) {
    setClimberLiftVoltage(
        MotorLim.clampVoltage(liftPID.calculate(inputs.liftMotorPosition, position)), inputs);
  }

  @Override
  public void setSolenoidState(boolean forward) {
    if (forward) {
      doubleSolenoidSim.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoidSim.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    liftSim.update(.02);
    inputs.liftMotorPosition = liftSim.getAngularPositionRotations();
    inputs.liftMotorVelocity = liftSim.getAngularVelocityRadPerSec();
    inputs.liftMotorCurrent = liftSim.getCurrentDrawAmps();
    inputs.liftMotorVoltage = liftSim.getInputVoltage();

    inputs.solenoidForward = doubleSolenoidSim.get() == DoubleSolenoid.Value.kForward;
    inputs.sensor1 = !sensor1Sim.get();
    inputs.sensor2 = !sensor2Sim.get();
  }
}
