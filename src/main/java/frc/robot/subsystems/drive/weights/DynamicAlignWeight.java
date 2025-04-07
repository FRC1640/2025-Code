package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;

public class DynamicAlignWeight implements DriveWeight {
  private FollowPathNearest globalAlign;
  private LocalTagAlignWeight localAlign;

  public DynamicAlignWeight(FollowPathNearest globalAlign, LocalTagAlignWeight localAlign) {
    this.globalAlign = globalAlign;
    this.localAlign = localAlign;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    if (localAlign.isReady()) {
      globalAlign.stopPath();
      return localAlign.getSpeeds();
    } else if (!globalAlign.isAutoalignComplete()) {
      if (!globalAlign.isEnabled()) {
        globalAlign.startPath();
      }
      return new ChassisSpeeds();
    } else {
      globalAlign.stopPath();
      return new ChassisSpeeds();
    }
  }

  public boolean globalAlignComplete() {
    return globalAlign.isAutoalignComplete();
  }

  @Override
  public void onFinish() {
    globalAlign.stopPath();
  }

  @Override
  public boolean isEnabled() {
    return DriveWeightCommand.checkWeight(this);
  }
}
