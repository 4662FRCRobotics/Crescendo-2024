// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoControl;
import frc.robot.Commands.AutoSelect;
import frc.robot.Commands.NoteIntake;
import frc.robot.Commands.SwerveDriveController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Libraries.ConsoleAuto;
import frc.robot.Subsystems.AutonomousSubsystem;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.AutonomousSubsystem.Paths;

/** Add your docs here. */
// If the climb has 2 independent arms it might be usfule to be able to
// indepently move them
// I dont want to have 2 driver controlers if not completly nessisary, last
// years two controllers were annoying.
public class RobotContainer {
    private final Drive m_drive = new Drive();
    private final Shooter m_Shooter = new Shooter();
    private final Intake m_intake = new Intake();

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDRIVER_CONTROLLER_PORT);

    private final ConsoleAuto m_consoleAuto = new ConsoleAuto(OperatorConstants.kAUTONOMOUS_CONSOLE_PORT);
    private final AutonomousSubsystem m_autonomous = new AutonomousSubsystem(m_consoleAuto, m_drive);

    private final AutoSelect m_autoSelect = new AutoSelect(m_autonomous);

    private final AutoControl m_autoCommand = new AutoControl(m_autonomous, m_drive);
    private SwerveDriveController m_swerveDriveController;

    public RobotContainer() {
        
        m_Shooter.setDefaultCommand(Commands.run(
                () -> m_Shooter.Shoot(MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), .5) * .5 + .5),
                m_Shooter));
                //sets the range you shoot at 1 to .75 controlled by how hard the trigger is pressed.


        m_drive.setDefaultCommand(
                Commands.run(
                        () -> m_drive.drive(
                                .5 * -MathUtil.applyDeadband(-m_driverController.getLeftX(), .08),
                                .5 * -MathUtil.applyDeadband(m_driverController.getLeftY(), .08),
                                .5 * -MathUtil.applyDeadband(m_driverController.getRightX(), .08), false,
                                true),
                        m_drive));

        configureBindings();

    }

    private void configureBindings() {
        var thetaController = new ProfiledPIDController(1, 0, 0, new Constraints(5, 1));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_swerveDriveController = new SwerveDriveController(
                m_autonomous.genTrajectory(Paths.BASIC), true, m_drive, thetaController);

        m_driverController
                .a()
                .onTrue(m_swerveDriveController);

        m_driverController
                .b()
                .whileTrue(new NoteIntake(m_intake)
                        .unless(() -> m_intake.isNoteIn())
                );

        m_driverController
                .x()
                .whileTrue(Commands.run(() -> m_intake.intakeNote(), m_intake)
                        .until(() -> m_intake.isNoteIn())
                        .andThen(() -> m_intake.stopIntake(), m_intake)
                        .unless(() -> m_intake.isNoteIn())
                );

        m_driverController
                .y()
                .whileTrue(Commands.sequence(
                        m_intake.deployIntakeEmpty(),
                        m_intake.intakeInLim(),
                        m_intake.parkIntakeLoaded()
                        )
                )
                ;
    }

    public Command getAutoSelect() {
        return m_autoSelect;
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // Command autoCommand = ;
        return m_autoCommand;
    }
}
