package frc.robot.subsystems.gantry;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.SparkConstants;

public class GantryIOSparkMax implements GantryIO {
  private final SparkMax carriageSpark;
  private final RelativeEncoder carriageEncoder;

  public GantryIOSparkMax() {
    carriageSpark = SparkConstants.getDefaultSparkMax(-1); // update with id
    carriageEncoder = carriageSpark.getEncoder();
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    // go through and update stuff
  }
}
