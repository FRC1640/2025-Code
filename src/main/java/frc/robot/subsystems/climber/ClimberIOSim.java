package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotPIDConstants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim motor1Sim; // lift
  private final DCMotorSim motor2Sim; // winch1
  private final DCMotorSim motor3Sim; // winch2
  private final DoubleSolenoidSim doubleSolenoidSim;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  PIDController liftController = RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);
  PIDController winchController = RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID);

  public ClimberIOSim() {
    DCMotor motor1SimGearbox = DCMotor.getNEO(1);
    DCMotor motor2SimGearbox = DCMotor.getNEO(1);
    DCMotor motor3SimGearbox = DCMotor.getNEO(1);

    motor1Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor1SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor1SimGearbox);
    motor2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor2SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor2SimGearbox);
    motor3Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor3SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor3SimGearbox);
    doubleSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);
  }

  /*
   * Sets the Lift Voltage
   */
  @Override
  public void setClimberLiftVoltage(double voltage, ClimberIOInputs inputs) {
    motor1Sim.setInputVoltage(clampVoltage(applyLimits(inputs.liftMotorPosition, voltage)));
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  @Override
  public void setClimberLiftPosition(double position, ClimberIOInputs inputs) {
    setClimberLiftVoltage(
        clampVoltage(liftController.calculate(inputs.liftMotorPosition, position)), inputs);
  }
  /*
   * Sets the Lift Voltage
   */
  @Override
  public void setClimberWinchVoltage(double voltage, ClimberIOInputs inputs) {
    motor2Sim.setInputVoltage(clampVoltage(applyLimits(inputs.winchMotor1Position, voltage)));
    motor3Sim.setInputVoltage(clampVoltage(applyLimits(inputs.winchMotor2Position, voltage)));
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  @Override
  public void setClimberWinchPosition(double position, ClimberIOInputs inputs) {
    setClimberWinchVoltage(
        clampVoltage(winchController.calculate(inputs.winchMotor1Position, position)), inputs);
  }

  /*
   * Set solenoid state (activated/not activated)
   */
  @Override
  public void setSolenoidState(boolean forward, ClimberIOInputs inputs) {
    if (forward) {
      doubleSolenoidSim.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoidSim.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    motor1Sim.update(.02);
    motor2Sim.update(.02);
    motor3Sim.update(.02);
    inputs.liftMotorPosition = motor1Sim.getAngularPositionRotations();
    inputs.winchMotor1Position = motor2Sim.getAngularPositionRotations();
    inputs.winchMotor2Position = motor3Sim.getAngularPositionRotations();
    inputs.liftMotorVelocity = motor1Sim.getAngularVelocityRadPerSec();
    inputs.winchMotor1Velocity = motor2Sim.getAngularVelocityRadPerSec();
    inputs.winchMotor2Velocity = motor3Sim.getAngularVelocityRadPerSec();
    inputs.liftMotorCurrent = motor1Sim.getCurrentDrawAmps();
    inputs.winchMotor1Current = motor2Sim.getCurrentDrawAmps();
    inputs.winchMotor2Current = motor3Sim.getCurrentDrawAmps();
    inputs.liftMotorVoltage = motor1Sim.getInputVoltage();
    inputs.winchMotor1Voltage = motor2Sim.getInputVoltage();
    inputs.winchMotor2Voltage = motor3Sim.getInputVoltage();
  }
}
