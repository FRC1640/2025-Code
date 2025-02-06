package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.tools.MotorLim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim intakemotor;
  final DoubleSolenoidSim doubleSolenoidSim;
  private final PIDController liftPID;

  public IntakeIOSim() {
    DCMotor intakemotor1SimGearbox = DCMotor.getNEO(1);
    this.intakemotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakemotor1SimGearbox, 0.00019125, RobotConstants.IntakeConstants.gearRatio),
            intakemotor1SimGearbox);
    this.doubleSolenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);
    this.liftPID = RobotPIDConstants.constructPID(RobotPIDConstants.intakePID);
  }

  @Override
  public void setIntakemotor1Voltage(final double voltage, IntakeIOInputs inputs) {
    intakemotor.setInputVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.intakeMotorPosition, voltage, IntakeConstants.intakeLimts)));
  }

  @Override
  public void setIntakemotor1Position(double position, IntakeIOInputs inputs) {
    setIntakemotor1Voltage(
        MotorLim.clampVoltage(liftPID.calculate(inputs.intakeMotorPosition, position)), inputs);
  }

  @Override
  public void setIntakeSolenoidState(final boolean forward, IntakeIOInputs inputs) {
    inputs.setSolenoidState(forward, this);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakemotor.update(0.02);
    inputs.intakeMotorCurrent = intakemotor.getCurrentDrawAmps();
    inputs.intakeMotorVoltage = intakemotor.getInputVoltage();
  }
}
