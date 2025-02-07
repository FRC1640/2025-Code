package frc.robot.subsystems.algaeintake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.util.tools.MotorLim;

public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
  private final DCMotorSim intakemotor;
  final DoubleSolenoidSim doubleSolenoidSim;

  public AlgaeIntakeIOSim() {
    DCMotor intakeMotorSimGearbox = DCMotor.getNEO(1);
    this.intakemotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeMotorSimGearbox, 0.00019125, RobotConstants.IntakeConstants.intakegearRatio),
            intakeMotorSimGearbox);
    this.doubleSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);
  }

  @Override
  public void setIntakeMotorVoltage(final double voltage, AlgaeIntakeIOInputs inputs) {
    intakemotor.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.intakeMotorPosition, voltage, IntakeConstants.intakeLimts)));
  }

  @Override
  public void setIntakeSolenoidState(final boolean forward, AlgaeIntakeIOInputs inputs) {
    inputs.setSolenoidState(forward, this);
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    intakemotor.update(0.02);
    inputs.intakeMotorCurrent = intakemotor.getCurrentDrawAmps();
    inputs.intakeMotorVoltage = intakemotor.getInputVoltage();
  }
}
