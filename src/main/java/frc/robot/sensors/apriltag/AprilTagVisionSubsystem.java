package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class AprilTagVisionSubsystem extends SubsystemBase {
  AprilTagVisionIO[] ios;
  AprilTagVisionIOInputsAutoLogged[] inputs;

  public AprilTagVisionSubsystem(AprilTagVisionIO... ios) {
    this.ios = ios;
    this.inputs = new AprilTagVisionIOInputsAutoLogged[ios.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }
  }

  public Rotation2d getTx(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Rotation2d getTy(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.ty();
  }

  public boolean isConnected(int cameraIndex) {
    return inputs[cameraIndex].connected;
  }

  public Integer[] getDisconnected() {
    ArrayList<Integer> indexes = new ArrayList<>();
    for (int i = 0; i < inputs.length; i++) {
      if (!inputs[i].connected) {
        indexes.add(i);
      }
    }
    return indexes.toArray(new Integer[indexes.size()]);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length && i < inputs.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera " + i, inputs[i]);
    }
  }
}
