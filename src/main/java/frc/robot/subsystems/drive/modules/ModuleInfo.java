package frc.robot.subsystems.drive.modules;

import frc.robot.constants.RobotConstants.PivotId;

//class to store constants for a module
public class ModuleInfo{
    
    PivotId id;
    int driveChannel;
    int steerChannel;
    int resolverChannel;
    double angleOffset;
    
    public ModuleInfo(PivotId id,
        int driveChannel,
        int steerChannel,
        int resolverChannel,
        double angleOffset){
            this.driveChannel = driveChannel;
            this.steerChannel = steerChannel;
            this.resolverChannel = resolverChannel;
            this.angleOffset = angleOffset;
        }
}