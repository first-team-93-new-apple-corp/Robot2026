// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.controls.ControllerSchemeIO;
import frc.robot.controls.TwoStickDrive;
import frc.robot.controls.TwoStickDriveXboxOp;
import frc.robot.controls.XboxDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Controls.Deadzone)
            .withRotationalDeadband(MaxAngularRate * Constants.Controls.Deadzone)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controls
    private final SendableChooser<String> controlSchemeChooser = new SendableChooser<>();
    private ControllerSchemeIO selectedControls;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TestSubsystem testSubsystem = new TestSubsystem(drivetrain);

    public RobotContainer() {
        /*
         * TODO:
         * - System Check
         * - Questnav
         * - modular?
         * - sysid routines
         * - Logging usb
         * - Constants
         */

        configureBindings();
    }

    private void updateControlScheme(String selected) {
        switch (selected) {
            case "TwoStickDrive":
                selectedControls = new TwoStickDrive(0, 1, 2);
                break;
            case "TwoStickDriveXboxOp":
                selectedControls = new TwoStickDriveXboxOp(0, 1, 2);
                break;
            case "XboxDrive":
            default:
                selectedControls = new XboxDrive(0);
                break;
        }
    }

    private void configureBindings() {
        // Controls
        controlSchemeChooser.setDefaultOption("Xbox Drive", "XboxDrive");
        controlSchemeChooser.addOption("Two Stick Drive", "TwoStickDrive");
        controlSchemeChooser.addOption("Two Stick + Xbox", "TwoStickDriveXboxOp");

        SmartDashboard.putData("Control Scheme", controlSchemeChooser);

        selectedControls = new XboxDrive(0);

        controlSchemeChooser.onChange(selected -> updateControlScheme(selected));

        // Swerve
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(selectedControls.DriveLeft() * MaxSpeed)
                        .withVelocityY(selectedControls.DriveUp() * MaxSpeed)
                        .withRotationalRate(selectedControls.DriveTheta() * MaxAngularRate)));

        // Sets to brake on disable
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> brake).ignoringDisable(true));
        // Set brownot voltage
        RobotController.setBrownoutVoltage(Volts.of(6));

        selectedControls.Brake().whileTrue(drivetrain.applyRequest(() -> brake));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        selectedControls.Back().and(selectedControls.Y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        selectedControls.Back().and(selectedControls.X()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        selectedControls.Menu().and(selectedControls.Y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        selectedControls.Menu().and(selectedControls.X()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        selectedControls.A().onTrue(startLogging());
        selectedControls.B().onTrue(stopLogging());

        // Reset the field-centric heading on left bumper press.
        selectedControls.Seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // selectedControls.Menu().onTrue(checkSwerve());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command checkSwerve() {
        return testSubsystem.commands.checkSwerve();
    }

    public Command stopLogging() {
        return Commands.runOnce(() -> SignalLogger.stop());
    }

    public Command startLogging() {
        SignalLogger.setPath("/media/sda1/testlogs/");
        return Commands.runOnce(() -> SignalLogger.start());
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                drivetrain.applyRequest(() -> brake));
    }
}
