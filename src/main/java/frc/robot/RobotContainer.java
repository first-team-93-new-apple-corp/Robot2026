// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.auto.AutoDirector;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    //Swerve
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(Constants.Swerve.MaxSpeed);

    // Controls
    private final SendableChooser<String> controlSchemeChooser = new SendableChooser<>();
    private ControllerSchemeIO selectedControls;

    // SysID
    private final SendableChooser<String> sysIDChooser = new SendableChooser<>();

    // Subystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TestSubsystem testSubsystem = new TestSubsystem(drivetrain);
    public final QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, new Pose3d());

    // Auto
    public final AutoDirector autoDirector = new AutoDirector();

    public RobotContainer() {
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
        SignalLogger.setPath("/media/sda1/logs/testlogs/");

        // Controls
        controlSchemeChooser.setDefaultOption("Xbox Drive", "XboxDrive");
        controlSchemeChooser.addOption("Two Stick Drive", "TwoStickDrive");
        controlSchemeChooser.addOption("Two Stick + Xbox", "TwoStickDriveXboxOp");

        SmartDashboard.putData("Control Scheme", controlSchemeChooser);

        selectedControls = new XboxDrive(0);

        controlSchemeChooser.onChange(selected -> updateControlScheme(selected));

        // Swerve
        drivetrain.setDefaultCommand(
                drivetrain.commands.applyRequest(() -> drive.withVelocityX(selectedControls.DriveLeft())
                        .withVelocityY(selectedControls.DriveUp())
                        .withRotationalRate(selectedControls.DriveTheta())));

        selectedControls.Seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        selectedControls.Brake().whileTrue(drivetrain.commands.applyRequest(() -> brake));

        RobotModeTriggers.disabled().whileTrue(
                drivetrain.commands.applyRequest(() -> brake).ignoringDisable(true));

        // Set brownot voltage
        RobotController.setBrownoutVoltage(Volts.of(6));

        // SysID
        sysIDChooser.setDefaultOption("Translation", "m_sysIdRoutineTranslation");
        sysIDChooser.addOption("Rotation", "m_sysIdRoutineRotation");
        sysIDChooser.addOption("Steer", "m_sysIdRoutineSteer");
        sysIDChooser.onChange(selected -> drivetrain.sysID.setSysIdRoutine(selected));
        SmartDashboard.putData("SysID", sysIDChooser);

        selectedControls.Back().and(selectedControls.Y())
                .toggleOnTrue(drivetrain.commands.sysIdDynamic(Direction.kForward));
        selectedControls.Back().and(selectedControls.X())
                .toggleOnTrue(drivetrain.commands.sysIdDynamic(Direction.kReverse));
        selectedControls.Menu().and(selectedControls.Y())
                .toggleOnTrue(drivetrain.commands.sysIdQuasistatic(Direction.kForward));
        selectedControls.Menu().and(selectedControls.X())
                .toggleOnTrue(drivetrain.commands.sysIdQuasistatic(Direction.kReverse));

        selectedControls.A().onTrue(startLogging());
        selectedControls.B().onTrue(stopLogging());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command stopLogging() {
        SmartDashboard.putBoolean("Logging?", false);
        return Commands.runOnce(SignalLogger::stop);
    }

    public Command startLogging() {
        SmartDashboard.putBoolean("Logging?", true);
        return Commands.runOnce(SignalLogger::start);
    }

    public Command getAutonomousCommand() {
        return Commands.defer(() -> autoDirector.selection().command(),
                Set.of(drivetrain, questNav));

    }
}
