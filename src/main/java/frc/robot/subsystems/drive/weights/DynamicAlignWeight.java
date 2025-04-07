package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;

public class DynamicAlignWeight implements DriveWeight {
  public enum AlignStage {
    kGlobal,
    kLocal,
    kDone
  }

  private FollowPathNearest globalAlign;
  private LocalTagAlignWeight localAlign;

  public DynamicAlignWeight(FollowPathNearest globalAlign, LocalTagAlignWeight localAlign) {
    this.globalAlign = globalAlign;
    this.localAlign = localAlign;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    switch (getStage()) {
      case kLocal:
        globalAlign.stopPath();
        return localAlign.getSpeeds();
      case kGlobal:
        if (!globalAlign.isEnabled()) {
          globalAlign.stopPath();
        } return new ChassisSpeeds();
      default:
        globalAlign.stopPath();
        return new ChassisSpeeds();
    }
  }

  public AlignStage getStage() {
    if (localAlign.isReady()) {
      return AlignStage.kLocal;
    } else if (!globalAlign.isAutoalignComplete()) {
      return AlignStage.kGlobal;
    } else {
      return AlignStage.kDone;
    }
  }

  public boolean isAutoalignComplete() {
    return globalAlignComplete() || localAlignComplete();
  }

  public boolean globalAlignComplete() {
    return globalAlign.isAutoalignComplete();
  }

  public boolean localAlignComplete() {
    return localAlign.isAutoalignComplete();
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
