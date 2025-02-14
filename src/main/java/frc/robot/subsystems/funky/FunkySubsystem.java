package frc.robot.subsystems.funky;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class FunkySubsystem extends SubsystemBase {
  FunkyIO funkyIO;
  BooleanSupplier funkyBooleanSupplier;
  BooleanSupplier theOtherFunkyBooleanSupplier;

  public FunkySubsystem(
      FunkyIO funky, BooleanSupplier funkySupplier, BooleanSupplier theOtherFunkySupplier) {
    funkyIO = funky;
    funkyBooleanSupplier = funkySupplier;
    theOtherFunkyBooleanSupplier = theOtherFunkySupplier;
  }

  @Override
  public void periodic() {
    System.out.println(funkyBooleanSupplier.getAsBoolean());
    System.out.println(funkyBooleanSupplier.getAsBoolean());
    if (funkyBooleanSupplier.getAsBoolean() == true) {
      funkyIO.setVelocity(3);
    }
    if (theOtherFunkyBooleanSupplier.getAsBoolean() == true) {
      funkyIO.setPos(10);
    }
  }
}
