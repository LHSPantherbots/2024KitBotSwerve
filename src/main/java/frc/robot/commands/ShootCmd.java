package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Launcher;

public class ShootCmd extends SequentialCommandGroup {

    public ShootCmd(Launcher launcher) {
        addCommands(
            new InstantCommand(() -> { launcher.setLauncher(-1.0);}),
            new WaitCommand(0.25),
            new InstantCommand(() -> { launcher.feed();}),
            new WaitCommand(0.25),
            new InstantCommand(() -> { launcher.StopAll();})
        );
    }
    
}
