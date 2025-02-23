package frc.robot.subsystems.lift;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.util.misc.MotorLim;
import frc.robot.constants.RobotPIDConstants;

import java.util.function.BooleanSupplier;

public class LiftIOSim implements LiftIO {
  private double velocitySetpoint = 0;
  private final DCMotorSim motor1Sim;
  private final DCMotorSim motor2Sim;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  private BooleanSupplier liftLimitSwitch;
  PIDController liftController =
      RobotPIDConstants.constructPID(RobotPIDConstants.liftPID, "LiftPID");
  ElevatorFeedforward elevatorFeedforward =
      RobotPIDConstants.constructFFElevator(RobotPIDConstants.liftFF);

  ProfiledPIDController profiledPIDController =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.liftProfiledPIDConstants, LiftConstants.constraints, "LiftPPID");
  private boolean limits;

  public LiftIOSim(BooleanSupplier liftLimitSwitch) {
    this.liftLimitSwitch = liftLimitSwitch;
    DCMotor motor1SimGearbox = DCMotor.getNEO(1);
    DCMotor motor2SimGearbox = DCMotor.getNEO(1);

    motor1Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor1SimGearbox, 0.00019125, LiftConstants.gearRatio),
            motor1SimGearbox);
    motor2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor2SimGearbox, 0.00019125, LiftConstants.gearRatio),
            motor2SimGearbox);
  }

  /*
   * Sets the Lift Voltage
   */
  @Override
  public void setLiftVoltage(double voltage, LiftIOInputs inputs) {
    motor1Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.leaderMotorPosition,
                voltage,
                limits ? LiftConstants.liftLimits.low : -99999,
                LiftConstants.liftLimits.high)));
    motor2Sim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.followerMotorPosition,
                voltage,
                limits ? LiftConstants.liftLimits.low : -99999,
                LiftConstants.liftLimits.high)));
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
    inputs.leaderMotorPosition =
        motor1Sim.getAngularPositionRad() * LiftConstants.sprocketRadius / LiftConstants.gearRatio;
    inputs.followerMotorPosition =
        motor2Sim.getAngularPositionRad() * LiftConstants.sprocketRadius / LiftConstants.gearRatio;
    inputs.leaderMotorVelocity =
        motor1Sim.getAngularVelocityRadPerSec()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio;
    inputs.followerMotorVelocity =
        motor2Sim.getAngularVelocityRadPerSec()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio;
    inputs.leaderMotorCurrent = motor1Sim.getCurrentDrawAmps();
    inputs.followerMotorCurrent = motor2Sim.getCurrentDrawAmps();
    inputs.leaderMotorVoltage = motor1Sim.getInputVoltage();
    inputs.followerMotorVoltage = motor2Sim.getInputVoltage();
    inputs.motorPosition = (inputs.leaderMotorPosition + inputs.followerMotorPosition) / 2;
    inputs.isLimitSwitchPressed = liftLimitSwitch.getAsBoolean();
  }

  @Override
  public void setLiftPositionMotionProfile(double position, LiftIOInputs inputs) {
    profiledPIDController.setGoal(position);
    setLiftVoltage(
        MotorLim.clampVoltage(
            profiledPIDController.calculate(inputs.leaderMotorPosition)
                + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity)),
        inputs);

    velocitySetpoint = profiledPIDController.getSetpoint().velocity;
  }

  @Override
  public void resetLiftMotionProfile(LiftIOInputs inputs) {
    profiledPIDController.reset(inputs.leaderMotorPosition);
  }

  @Override
  public double velocitySetpoint() {
    return velocitySetpoint;
  }

  @Override
  public void setLimitEnabled(boolean enable) {
    limits = enable;
  }
}
