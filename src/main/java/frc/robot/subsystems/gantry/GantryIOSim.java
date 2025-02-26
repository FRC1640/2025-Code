package frc.robot.subsystems.gantry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;
import java.util.function.BooleanSupplier;

public class GantryIOSim implements GantryIO {
  private double velocitySetpoint = 0;
  private final DCMotorSim gantrySim;
  private double gantryAppliedVolts = 0.0;
  private BooleanSupplier gantryLimitSwitch;
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID, "gantryPID");
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.gantryFF, "gantryFF");
  private final PIDController gantryVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryVelocityPID, "gantryVelocityPID");
  private boolean limits = false;
  private final ProfiledPIDController gantryPPID =
      RobotPIDConstants.constructProfiledPIDController(
          RobotPIDConstants.gantryProfiledPID, GantryConstants.constraints);

  public GantryIOSim(BooleanSupplier gantryLimitSwitch) {
    this.gantryLimitSwitch = gantryLimitSwitch;
    DCMotor gantryGearbox = DCMotor.getNeo550(1); // 550 confirmed
    gantrySim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gantryGearbox, 0.00019125, 1), gantryGearbox);
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    gantrySim.update(.02);

    inputs.currentAmps = gantrySim.getCurrentDrawAmps();
    inputs.encoderPosition =
        gantrySim.getAngularPositionRad()
            / GantryConstants.gantryGearRatio
            * GantryConstants.pulleyRadius;
    inputs.isLimitSwitchPressed = gantryLimitSwitch.getAsBoolean();
    inputs.appliedVoltage = gantrySim.getInputVoltage();
    inputs.encoderVelocity =
        gantrySim.getAngularVelocityRadPerSec()
            / GantryConstants.gantryGearRatio
            * GantryConstants.pulleyRadius;
  }

  @Override
  public void setGantryPosition(double pos, GantryIOInputs inputs) {
    setGantryVoltage(
        MotorLim.clampVoltage(gantryPID.calculate(inputs.encoderPosition, pos)), inputs);
  }

  @Override
  public void setGantryVoltage(double voltage, GantryIOInputs inputs) {
    gantrySim.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.encoderPosition,
                voltage,
                GantryConstants.gantryLimits.low,
                limits ? GantryConstants.gantryLimits.high : 999999)));
  }

  @Override
  public void setGantryPositionMotionProfile(double position, GantryIOInputs inputs) {
    gantryPPID.setGoal(position);
    setGantryVoltage(
        MotorLim.clampVoltage(
            gantryPPID.calculate(inputs.encoderPosition)
                + ff.calculate(gantryPPID.getSetpoint().velocity)),
        inputs);

    velocitySetpoint = gantryPPID.getSetpoint().velocity;
  }

  @Override
  public void resetGantryMotionProfile(GantryIOInputs inputs) {
    gantryPPID.reset(inputs.encoderPosition);
  }

  @Override
  public void setLimitEnabled(boolean enable) {
    limits = enable;
  }

  @Override
  public void setGantryVelocity(double velocity, GantryIOInputs inputs) {
    setGantryVoltage(
        ff.calculate(velocity) + gantryVelocityPID.calculate(inputs.encoderVelocity, velocity),
        inputs);
    velocitySetpoint = velocity;
  }

  @Override
  public double velocitySetpoint() {
    return velocitySetpoint;
  }
}
