package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.misc.EMA;
import frc.robot.util.misc.MotorLim;
import frc.robot.util.spark.SparkConfigurer;
import org.littletonrobotics.junction.Logger;

public class LiftIOSpark implements LiftIO {
  private double velocitySetpoint = 0;
  RelativeEncoder leaderEncoder;
  RelativeEncoder followerEncoder;
  SparkMax leaderMotor;
  SparkMax followerMotor;
  SparkLimitSwitch liftLimitSwitch; // direction?? the gantry one didn't have it specified
  PIDController liftController =
      RobotPIDConstants.constructPID(RobotPIDConstants.liftPID, "LiftPID");
  ElevatorFeedforward elevatorFeedforward =
      RobotPIDConstants.constructFFElevator(RobotPIDConstants.liftFF);
  double lastTime = Timer.getFPGATimestamp();

  ProfiledPIDController profiledPIDController =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.liftProfiledPIDConstants, LiftConstants.constraints, "LiftPPID");

  PIDController velocityController =
      RobotPIDConstants.constructPID(RobotPIDConstants.liftVelocityPID, "LiftVelocityPID");
  private boolean limits;
  private EMA EMACurrent;

  public LiftIOSpark() {
    leaderMotor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getLiftDefaultMax(LiftConstants.liftLeaderMotorID, false)
                .applyPIDConfig(RobotPIDConstants.pidConstantSpark));
    followerMotor =
        SparkConfigurer.configSparkMax(
            SparkConstants.getLiftDefaultMax(LiftConstants.liftFollowerMotorID, true), leaderMotor);
    leaderEncoder = leaderMotor.getEncoder();
    followerEncoder = followerMotor.getEncoder();
    liftLimitSwitch = leaderMotor.getReverseLimitSwitch();
    profiledPIDController.setTolerance(0.003);
    EMACurrent = new EMA(LiftConstants.emaSmoothing, LiftConstants.emaPeriod);
  }
  /*
   * Set voltage of the motor
   */
  @Override
  public void setLiftVoltage(double voltage, LiftIOInputs inputs) {
    double clampedVoltage;
    if (EMACurrent.get() < LiftConstants.currentThresh) {
      clampedVoltage = voltage;
    } else {
      clampedVoltage = 0;
    }

    Logger.recordOutput("LIFTINPUT", clampedVoltage);
    Logger.recordOutput(
        "liftinputclamped",
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.leaderMotorPosition,
                clampedVoltage,
                limits ? LiftConstants.liftLimits.low : -99999,
                LiftConstants.liftLimits.high)));
    leaderMotor.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.leaderMotorPosition,
                clampedVoltage,
                limits ? LiftConstants.liftLimits.low : -99999,
                LiftConstants.liftLimits.high)));
  }
  /*
   * Sets the position of the motor(s) using a PID
   */
  @Override
  public void setLiftPosition(double position, LiftIOInputs inputs) {
    Logger.recordOutput("pos", position);
    setLiftVoltage(
        MotorLim.clampVoltage(
            liftController.calculate(inputs.leaderMotorPosition, position)
                + elevatorFeedforward.calculate(0)),
        inputs);
  }

  @Override
  public void resetLiftPositionPid() {
    liftController.reset();
  }

  @Override
  public void setLiftPositionMotionProfile(double position, LiftIOInputs inputs) {
    profiledPIDController.setGoal(position);
    Logger.recordOutput(
        "LiftFeedForward",
        elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity));
    setLiftVoltage(
        MotorLim.clampVoltage(
            profiledPIDController.calculate(inputs.leaderMotorPosition, position)
                + elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity)
                + velocityController.calculate(
                    inputs.leaderMotorVelocity, profiledPIDController.getSetpoint().velocity)),
        inputs);
    velocitySetpoint = profiledPIDController.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void resetLiftMotionProfile(LiftIOInputs inputs) {
    profiledPIDController.reset(inputs.leaderMotorPosition, inputs.leaderMotorVelocity);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.leaderMotorPosition =
        leaderEncoder.getPosition()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2;
    inputs.followerMotorPosition =
        followerEncoder.getPosition()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2;
    inputs.leaderMotorVelocity =
        leaderEncoder.getVelocity()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2
            / 60;
    inputs.followerMotorVelocity =
        followerEncoder.getVelocity()
            * LiftConstants.sprocketRadius
            / LiftConstants.gearRatio
            * Math.PI
            * 2
            / 60;
    inputs.leaderMotorCurrent = leaderMotor.getOutputCurrent();
    inputs.followerMotorCurrent = followerMotor.getOutputCurrent();
    inputs.leaderMotorVoltage =
        leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.followerMotorVoltage =
        followerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.leaderTemperature = leaderMotor.getMotorTemperature();
    inputs.followerTemperature = followerMotor.getMotorTemperature();
    inputs.motorPosition = (inputs.leaderMotorPosition + inputs.followerMotorPosition) / 2;
    inputs.isLimitSwitchPressed = liftLimitSwitch.isPressed();
  }

  @Override
  public void resetEncoder() {
    leaderEncoder.setPosition(0);
    followerEncoder.setPosition(0);
  }

  @Override
  public double velocitySetpoint() {
    return velocitySetpoint;
  }

  @Override
  public void setLimitEnabled(boolean enable) {
    limits = enable;
  }

  @Override
  public void testMethod() {
    profiledPIDController =
        RobotPIDConstants.constructProfiledPIDController(
            RobotPIDConstants.liftProfiledPIDConstants, LiftConstants.constraints, "LiftPPID");
  }

  @Override
  public void updateEMA(double data) {
    EMACurrent.update(data);
  }
}
