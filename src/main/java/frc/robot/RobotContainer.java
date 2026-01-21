// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick leftJoystick = new CommandJoystick(0);
    private final CommandJoystick rightJoystick = new CommandJoystick(1);

    private final CommandXboxController xBoxController = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

    public final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand( // For Xbox Controller
        // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(-xBoxController.getLeftY() * MaxSpeed) // Drive forward
        // with negative Y (forward)
        // .withVelocityY(-xBoxController.getLeftX() * MaxSpeed) // Drive left with
        // negative X (left)
        // .withRotationalRate(-xBoxController.getRightX() * MaxAngularRate) // Drive
        // counterclockwise with negative X (left)
        // )
        // );
        drivetrain.setDefaultCommand( // For Joysticks
                drivetrain.applyRequest(() -> drive.withVelocityX(-leftJoystick.getY() * MaxSpeed) // Drive forward with
                                                                                                   // negative
                                                                                                   // Y(forward)
                        .withVelocityY(-leftJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-rightJoystick.getX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        xBoxController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        xBoxController.leftBumper().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-xBoxController.getLeftY(), -xBoxController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xBoxController.back().and(xBoxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        xBoxController.back().and(xBoxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        xBoxController.start().and(xBoxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        xBoxController.start().and(xBoxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        leftJoystick.button(12).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        xBoxController.b().onTrue(m_IntakeSubsystem.Commands.intake());
        xBoxController.b().onFalse(m_IntakeSubsystem.Commands.stop());

        xBoxController.x().onTrue(m_IntakeSubsystem.Commands.outtake());
        xBoxController.x().onFalse(m_IntakeSubsystem.Commands.stop());

        xBoxController.y().onTrue(m_ClimberSubsystem.Commands.Retract());
        xBoxController.y().onFalse(m_ClimberSubsystem.Commands.Stop());

        xBoxController.a().onTrue(m_ClimberSubsystem.Commands.Extend());
        xBoxController.a().onFalse(m_ClimberSubsystem.Commands.Stop());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
