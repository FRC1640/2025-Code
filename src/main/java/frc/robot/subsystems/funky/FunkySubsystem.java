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
    if (funkyBooleanSupplier.getAsBoolean() == true) {
      funkyIO.setVelocity(3);
    } else {
      funkyIO.setVelocity(0);
    }
    if (theOtherFunkyBooleanSupplier.getAsBoolean() == true) {
      funkyIO.setPos(10);
    }
    else{
      funkyIO.setPos(1);
    }
  }
}
