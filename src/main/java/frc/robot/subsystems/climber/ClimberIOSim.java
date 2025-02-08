package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;
import java.util.function.BooleanSupplier;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim liftSim;
  private final DCMotorSim winch1Sim; // winch leader motor
  private final DCMotorSim winch2Sim; // winch follower motor
  private final DoubleSolenoidSim doubleSolenoidSim;
  private final PIDController liftPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID);
  BooleanSupplier cageDetect;

  public ClimberIOSim() {
    DCMotor motor1SimGearbox = DCMotor.getNEO(1);
    DCMotor motor2SimGearbox = DCMotor.getNEO(1);
    DCMotor motor3SimGearbox = DCMotor.getNEO(1);

    liftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor1SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor1SimGearbox);
    winch1Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor2SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor2SimGearbox);
    winch2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor3SimGearbox, 0.00019125, RobotConstants.ClimberConstants.gearRatio),
            motor3SimGearbox);
    doubleSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);
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
  public void setClimberWinchVoltage(double voltage, ClimberIOInputs inputs) {
    winch1Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchLeaderMotorPosition, voltage, ClimberConstants.winchLimits)));
    winch2Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchFollowerMotorPosition, voltage, ClimberConstants.winchLimits)));
  }

  @Override
  public void setClimberWinchPosition(double position, ClimberIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchPID.calculate(inputs.winchLeaderMotorPosition, position)),
        inputs);
  }

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
    liftSim.update(.02);
    winch1Sim.update(.02);
    winch2Sim.update(.02);
    inputs.cageDetected = cageDetect.getAsBoolean();
    inputs.liftMotorPosition = liftSim.getAngularPositionRotations();
    inputs.winchLeaderMotorPosition = winch1Sim.getAngularPositionRotations();
    inputs.winchFollowerMotorPosition = winch2Sim.getAngularPositionRotations();
    inputs.liftMotorVelocity = liftSim.getAngularVelocityRadPerSec();
    inputs.winchLeaderMotorVelocity = winch1Sim.getAngularVelocityRadPerSec();
    inputs.winchFollowerMotorVelocity = winch2Sim.getAngularVelocityRadPerSec();
    inputs.liftMotorCurrent = liftSim.getCurrentDrawAmps();
    inputs.winchLeaderMotorCurrent = winch1Sim.getCurrentDrawAmps();
    inputs.winchFollowerMotorCurrent = winch2Sim.getCurrentDrawAmps();
    inputs.liftMotorVoltage = liftSim.getInputVoltage();
    inputs.winchLeaderMotorVoltage = winch1Sim.getInputVoltage();
    inputs.winchFollowerMotorVoltage = winch2Sim.getInputVoltage();

    inputs.solenoidForward = doubleSolenoidSim.get() == DoubleSolenoid.Value.kForward;
  }
}
