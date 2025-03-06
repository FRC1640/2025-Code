package frc.robot.subsystems.coralouttake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import java.util.function.BooleanSupplier;

public class CoralOuttakeIOSim implements CoralOuttakeIO {
  private final DCMotorSim intakeSim;
  private double appliedVoltage;
  BooleanSupplier coralDetect;
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.coralFF, "coralFF");
  private final PIDController coralVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.coralVelocityPID, "coralVelocityPID");

  public CoralOuttakeIOSim(BooleanSupplier coralDetect) {
    DCMotor intakeGearbox = DCMotor.getNeo550(1);
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeGearbox, 0.00019125, GantryConstants.gantryGearRatio),
            intakeGearbox);

    this.coralDetect = coralDetect;
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    intakeSim.setInputVoltage(appliedVoltage);
    intakeSim.update(.02);
    inputs.coralDetectedHigh = coralDetect.getAsBoolean();
    inputs.hasCoral = coralDetect.getAsBoolean();
    inputs.appliedVoltage = appliedVoltage;
    inputs.outtakeVelocity = intakeSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    appliedVoltage = voltage;
  }

  @Override
  public void setIntakeVelocity(double velocity, CoralOuttakeIOInputs inputs) {
    setIntakeVoltage(
        ff.calculate(velocity) + coralVelocityPID.calculate(inputs.outtakeVelocity, velocity));
  }
}
