package frc.robot.subsystems.lift;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;

public class LiftIOSim implements LiftIO {
  private final DCMotorSim motor1Sim;
  private final DCMotorSim motor2Sim;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  PIDController liftController = RobotPIDConstants.constructPID(RobotPIDConstants.liftPID);

  public LiftIOSim() {
    DCMotor motor1SimGearbox = DCMotor.getNEO(1);
    DCMotor motor2SimGearbox = DCMotor.getNEO(1);

    motor1Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor1SimGearbox, 0.00019125, RobotConstants.LiftConstants.gearRatio),
            motor1SimGearbox);
    motor2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor2SimGearbox, 0.00019125, RobotConstants.LiftConstants.gearRatio),
            motor2SimGearbox);
  }

  /*
   * Sets the Lift Voltage
   */
  @Override
  public void setLiftVoltage(double voltage, LiftIOInputs inputs) {
    motor1Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(inputs.leaderMotorPosition, voltage, LiftConstants.liftLimits)));
    motor2Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(inputs.followerMotorPosition, voltage, LiftConstants.liftLimits)));
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  @Override
  public void setLiftPosition(double position, LiftIOInputs inputs) {
    setLiftVoltage(
        MotorLim.clampVoltage(liftController.calculate(inputs.leaderMotorPosition, position)),
        inputs);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    motor1Sim.update(.02);
    motor2Sim.update(.02);
    inputs.leaderMotorPosition = motor1Sim.getAngularPositionRotations();
    inputs.followerMotorPosition = motor2Sim.getAngularPositionRotations();
    inputs.leaderMotorVelocity = motor1Sim.getAngularVelocityRadPerSec();
    inputs.followerMotorVelocity = motor2Sim.getAngularVelocityRadPerSec();
    inputs.leaderMotorCurrent = motor1Sim.getCurrentDrawAmps();
    inputs.followerMotorCurrent = motor2Sim.getCurrentDrawAmps();
    inputs.leaderMotorVoltage = motor1Sim.getInputVoltage();
    inputs.followerMotorVoltage = motor2Sim.getInputVoltage();
  }
}
