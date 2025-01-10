package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.sysid.CreateSysidCommand;



public class DashboardInit {
    /*
     * TODO: dashboard with elastic
     * 
     * should include:
     * - sysid setup
     * - pid tuner? maybe?
     * - auto setup
     * - in match dashboard
     */
    double testNumber = 12345;
    static boolean testBoolean = true;
    private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
    //private static DriveSubsystem driveSubsystem;
    private static CommandXboxController controller;
    public static void testTestInit(){
        sysidInit();;
    }

    private static void sysidInit(){
        ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
        sysidChooser.setDefaultOption("none :/", new WaitCommand(0.1));
        // sysidChooser.addOption("SwerveSysID",
                // CreateSysidCommand.createCommand(driveSubsystem::sysIdQuasistatic, driveSubsystem::sysIdDynamic,
                        // "SwerveSysId",controller, ()->driveSubsystem.stopMotors()));
        sysidTab.add(sysidChooser).withSize(5,5).withPosition(1,1);
    }
}
