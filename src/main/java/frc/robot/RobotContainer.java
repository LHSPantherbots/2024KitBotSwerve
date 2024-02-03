// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePadButtons;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShootCmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.RobotStateSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final RobotStateSubsystem robotState = new RobotStateSubsystem();
    // public static final LimeLight limelight = new LimeLight();
    public static final DriveSubsystem driveTrain = new DriveSubsystem();
    public static final Launcher launcher = new Launcher();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

   // public static SendableChooser<Command> autoChoice = new SendableChooser<>();
   private final SendableChooser<Command> autoChoice;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        NamedCommands.registerCommand("ShootCmd", new ShootCmd(launcher));

        autoChoice = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Autonomous").add(autoChoice);
        autoChoice.addOption("Do Nothing", new RunCommand(() -> driveTrain.drive(0, 0, 0, true)));

        // Configure the button bindings
        configureButtonBindings();

        // Set the default drive command to split-stick arcade drive
        driveTrain.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> driveTrain.driveRel(
                                -m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond,
                                -m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond,
                                // -m_driverController.getRightX()
                                -(m_driverController.getRightTriggerAxis()
                                        - m_driverController.getLeftTriggerAxis())
                                        * DriveConstants.kMaxSpeedMetersPerSecond
                                ),
                        driveTrain));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // DRIVER CONTROLS

        new JoystickButton(m_driverController, GamePadButtons.Start)
                .whileTrue(new InstantCommand(driveTrain::resetAll, driveTrain));
        new JoystickButton(m_driverController, GamePadButtons.Select)
                .whileTrue(new InstantCommand(driveTrain::restOdomWithCamData));
        // While the left bumper is held down, the robot's speed will be set to a tenth
        // of its standard
        // value,
        // and the leds will pulse orange to indicate reduced speed
        // double reducedspeedconstant = 0.2; // how much the speed is reduced by
        // new JoystickButton(m_driverController, GamePadButtons.LB)
        // .whileTrue(
        // new RunCommand(
        // () ->
        // driveTrain.drive(
        // -m_driverController.getLeftY()
        // * reducedspeedconstant
        // * DriveConstants.kMaxSpeedMetersPerSecond,
        // -m_driverController.getLeftX()
        // * reducedspeedconstant
        // * DriveConstants.kMaxSpeedMetersPerSecond,
        // // -m_driverController.getRightX()
        // -(m_driverController.getRightTriggerAxis()
        // - m_driverController.getLeftTriggerAxis())
        // * reducedspeedconstant
        // * 2.0
        // * DriveConstants
        // .kMaxSpeedMetersPerSecond, // Doubled the rotation because it was
        // // not turning at reduced speed
        // true),
        // driveTrain))
        // .whileTrue(new RunCommand(leds::orangePulse, leds));

        // Operator Controls

        new JoystickButton(operatorController, GamePadButtons.B)
                .whileTrue(new RunCommand(launcher::intake, launcher))
                .onFalse(new InstantCommand(launcher::newResume, launcher));
        // new JoystickButton(operatorController, GamePadButtons.B)
        // .onTrue(new InstantCommand(launcher::newIntake, launcher))
        // .onFalse(new InstantCommand(launcher::newResume, launcher));

        new JoystickButton(m_driverController, GamePadButtons.LB)
                .whileTrue(new InstantCommand(driveTrain::setRelitive, driveTrain));     

        new JoystickButton(operatorController, GamePadButtons.X)
                .whileTrue(new InstantCommand(launcher::StopAll, launcher));

        new JoystickButton(operatorController, GamePadButtons.A)
                .whileTrue(new RunCommand(launcher::feed, launcher))
                .onFalse(new InstantCommand(() -> launcher.setFeed(0)));

        new POVButton(operatorController, GamePadButtons.Up)
                .onTrue(new InstantCommand(launcher::launcherRpmUp, launcher));

        new POVButton(operatorController, GamePadButtons.Down)
                .onTrue(new InstantCommand(launcher::launcherRpmDown, launcher));

        // new POVButton(operatorController, GamePadButtons.Left)
        // .onTrue(new InstantCommand(launcher::launcherStop, launcher));

        // new POVButton(operatorController, GamePadButtons.Right)
        // .onTrue(new InstantCommand(launcher::launcherResume, launcher));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChoice.getSelected();
    }
}
