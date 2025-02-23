package frc.robot.subsystems.winch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.misc.MotorLim;

public class WinchIOSim implements WinchIO {
  private final DCMotorSim winch1Sim; // winch leader motor
  private final DCMotorSim winch2Sim; // winch follower motor
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID, "winchPID");

  public WinchIOSim() {
    DCMotor motor2SimGearbox = DCMotor.getNEO(1);
    DCMotor motor3SimGearbox = DCMotor.getNEO(1);
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
  }

  @Override
  public void setClimberWinchVoltage(double voltage, WinchIOInputs inputs) {
    winch1Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchLeaderMotorPosition,
                voltage,
                ClimberConstants.winchLimits.low,
                ClimberConstants.winchLimits.high)));
    winch2Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchFollowerMotorPosition,
                voltage,
                ClimberConstants.winchLimits.low,
                ClimberConstants.winchLimits.high)));
  }

  @Override
  public void setClimberWinchPosition(double position, WinchIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchPID.calculate(inputs.winchLeaderMotorPosition, position)),
        inputs);
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    winch1Sim.update(.02);
    winch2Sim.update(.02);
    inputs.winchLeaderMotorPosition = winch1Sim.getAngularPositionRotations();
    inputs.winchFollowerMotorPosition = winch2Sim.getAngularPositionRotations();
    inputs.winchLeaderMotorVelocity = winch1Sim.getAngularVelocityRadPerSec();
    inputs.winchFollowerMotorVelocity = winch2Sim.getAngularVelocityRadPerSec();
    inputs.winchLeaderMotorCurrent = winch1Sim.getCurrentDrawAmps();
    inputs.winchFollowerMotorCurrent = winch2Sim.getCurrentDrawAmps();
    inputs.winchLeaderMotorVoltage = winch1Sim.getInputVoltage();
    inputs.winchFollowerMotorVoltage = winch2Sim.getInputVoltage();
  }
}
