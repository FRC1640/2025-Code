package frc.robot.subsystems.gantry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;

public class GantryIOSim implements GantryIO {
  private final DCMotorSim gantrySim;
  private double gantryAppliedVolts = 0.0;
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID, "gantryPID");

  public GantryIOSim() {
    DCMotor gantryGearbox = DCMotor.getNeo550(1); // 550 confirmed
    gantrySim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gantryGearbox, 0.00019125, GantryConstants.gantryGearRatio),
            gantryGearbox);
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    gantrySim.update(.02);

    inputs.currentAmps = gantrySim.getCurrentDrawAmps();
    inputs.encoderPosition = gantrySim.getAngularPositionRad();
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
            MotorLim.applyLimits(inputs.encoderPosition, voltage, GantryConstants.gantryLimits)));
  }
}
