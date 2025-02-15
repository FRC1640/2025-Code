package frc.robot.subsystems.gantry;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;
import java.util.function.BooleanSupplier;

public class GantryIOSim implements GantryIO {
  private double velocitySetpoint = 0;
  private double gantryAppliedVolts = 0.0;
  private BooleanSupplier gantryLimitSwitch;
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID, "gantryPID");
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.gantryFF);
  private final PIDController gantryVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryVelocityPID);
  public SparkMax gantryMax;
  public SparkMaxSim gantrySim;
  public SparkRelativeEncoderSim gantryEncoder;

  public GantryIOSim(BooleanSupplier gantryLimitSwitch) {
    this.gantryLimitSwitch = gantryLimitSwitch;
    DCMotor gantryGearBox = DCMotor.getNeo550(1);
    gantryMax = new SparkMax(GantryConstants.gantrySparkID, MotorType.kBrushless);
    gantrySim = new SparkMaxSim(gantryMax, gantryGearBox);
    gantryEncoder = gantrySim.getRelativeEncoderSim();
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    gantrySim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(gantrySim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        0.02);
    inputs.currentAmps = gantrySim.getMotorCurrent();
    inputs.encoderPosition =
        gantrySim.getPosition() / GantryConstants.gantryGearRatio * GantryConstants.pulleyRadius;
    inputs.isLimitSwitchPressed = gantryLimitSwitch.getAsBoolean();
    inputs.appliedVoltage = gantrySim.getBusVoltage();
    inputs.encoderVelocity =
        gantrySim.getVelocity() / GantryConstants.gantryGearRatio * GantryConstants.pulleyRadius;
  }

  @Override
  public void setGantryPosition(double pos, GantryIOInputs inputs) {
    setGantryVoltage(
        MotorLim.clampVoltage(gantryPID.calculate(inputs.encoderPosition, pos)), inputs);
  }

  @Override
  public void setGantryVoltage(double voltage, GantryIOInputs inputs) {
    gantrySim.setBusVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.encoderPosition,
                voltage,
                GantryConstants.gantryLimits.low,
                inputs.isLimitSwitchPressed)));
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
