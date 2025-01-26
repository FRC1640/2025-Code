package frc.robot.subsystems.gantry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;

public class GantryIOSim implements GantryIO {
  private final DCMotorSim carriageSim;
  private double gantryAppliedVolts = 0.0;
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID);

  public GantryIOSim() {
    DCMotor gantryGearbox = DCMotor.getNeo550(1); // 550 confirmed
    carriageSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gantryGearbox, 0, GantryConstants.gantryGearRatio),
            gantryGearbox);
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    inputs.currentAmps = carriageSim.getCurrentDrawAmps();
    inputs.appliedVoltage = gantryAppliedVolts;

    carriageSim.setInputVoltage(gantryAppliedVolts);
    carriageSim.update(.02);
  }

  @Override
  public void setCarriagePosition(double pos, GantryIOInputs inputs) {
    setGantrySpeedVoltage(
        MathUtil.clamp(gantryPID.calculate(inputs.encoderPosition, pos) * 12, -12, 12));
  }

  @Override
  public void setGantrySpeedVoltage(double voltage) {
    gantryAppliedVolts = voltage;
  }
}
