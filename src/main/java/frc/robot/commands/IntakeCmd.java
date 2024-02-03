package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Launcher;

public class IntakeCmd extends SequentialCommandGroup {

    public IntakeCmd(Launcher launcher) {
        addCommands(
            new InstantCommand(() -> { launcher.intake();}),
            new WaitCommand(1),
            new InstantCommand(() -> { launcher.StopAll();})
        );
    }
    
}