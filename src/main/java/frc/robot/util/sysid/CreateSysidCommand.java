package frc.robot.util.sysid;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CreateSysidCommand {
    /**
     * 
     * @param quasistaticCommand
     * @param dynamicCommand
     * @param commandName
     * @param startNext starts next phase of routine when this is triggered
     * @param cancel pauses sysid routine when this is triggered
     * @param stopMotors
     * @return complete sysid command
     */
    public static Command createCommand(Function<SysIdRoutine.Direction, Command>  quasistaticCommand, Function<SysIdRoutine.Direction, Command>  dynamicCommand,
            String commandName, BooleanSupplier startNext, BooleanSupplier cancel, Runnable stopMotors) {
        Command sysIdCommand = new SequentialCommandGroup(
                quasistaticCommand.apply(SysIdRoutine.Direction.kForward)
                        .finallyDo(()->stopMotors.run()).until(cancel),
                new WaitUntilCommand(startNext),
                quasistaticCommand.apply(SysIdRoutine.Direction.kReverse)
                        .finallyDo(()->stopMotors.run()).until(cancel),
                new WaitUntilCommand(startNext),
                dynamicCommand.apply(SysIdRoutine.Direction.kForward)
                        .finallyDo(()->stopMotors.run()).until(cancel),
                new WaitUntilCommand(startNext),
                dynamicCommand.apply(SysIdRoutine.Direction.kReverse)
                       .finallyDo(()->stopMotors.run()).until(cancel));
        sysIdCommand.setName(commandName);
        return sysIdCommand;
    }
}
