package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

    public static void testTestInit(){
        testInit();
    }

    private static void testInit(){
        ShuffleboardTab testingaaah = Shuffleboard.getTab("testingaaah");
        testingaaah.addBoolean("test??", () -> testBoolean).withSize(1,1).withPosition(0, 0);
    }

    private static void sysidInit(){
        ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
        sysidChooser.setDefaultOption("none :/", new WaitCommand(0.1));
        sysidChooser.addOption("Swerve", CreateSysidCommand.createCommand(driveSubsystem))
    }
    // private static void testInitButNamedBob(){
    //     Elastic.selectTab("bob");
    // }
    //make sure inits to default layout 
        //seems like we might be able to "load layout from robot" (once we have a robot) but auto boot would be nice

        /* mechanical advantage 2024 thing
         * public AutoSelector(String key) {
    routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;
    lastResponses = List.of();

    // Publish questions and choosers
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
    }
         */
}
