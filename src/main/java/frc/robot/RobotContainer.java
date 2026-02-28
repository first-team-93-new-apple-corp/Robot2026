// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Controls.ControllerSchemeIO;
import frc.robot.Controls.TwoStickDriveXboxOp;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.generated.TunerConstants;

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

    private final ControllerSchemeIO driver = new TwoStickDriveXboxOp(0, 1, 2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

    public final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

    public final ManipulationSubsystem m_ManipulationSubsystem = new ManipulationSubsystem();

    public RobotContainer() {

        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.DriveLeft() * MaxSpeed) // Drive forward with
                                                                                                  // negative Y
                                                                                                  // (forward)
                        .withVelocityY(-driver.DriveUp() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.DriveTheta() * MaxAngularRate) // Drive counterclockwise with
                                                                                   // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.brake().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.brake().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.InputUp(), -driver.InputLeft()))));

        driver.Intake().onTrue(m_ManipulationSubsystem.commands.intakeCommand());
        driver.Outtake().onTrue(m_ManipulationSubsystem.commands.outtakeCommand());
        driver.Intake().and(driver.Outtake()).onFalse(m_ManipulationSubsystem.commands.idleCommand());

        // Reset the field-centric heading on left bumper press.
        driver.seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        driver.autoRetractClimber().onTrue(m_ClimberSubsystem.commands.autoRetract());

        driver.autoExtendClimber().onTrue(m_ClimberSubsystem.commands.autoExtend());
        driver.manExtendClimber().onTrue(m_ClimberSubsystem.commands.manualExtend());
        driver.manExtendClimber().onFalse(m_ClimberSubsystem.commands.Stop());
        driver.manRetractClimber().onTrue(m_ClimberSubsystem.commands.manualRetract());
        driver.manRetractClimber().onFalse(m_ClimberSubsystem.commands.Stop());
        driver.resetClimberEncoder().onTrue(m_ClimberSubsystem.commands.resetEncoder());
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
