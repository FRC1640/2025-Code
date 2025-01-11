package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveWeight {
  public default ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds();
  }
  ;

  public default double getWeight() {
    return 1;
  }
  ;

  public default void setWeight(double weight) {}
  ;

  public default boolean cancelCondition() {
    return false;
  }
  ;
}
