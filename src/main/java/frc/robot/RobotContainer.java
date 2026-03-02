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
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                        // top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone) // Add a
                                                                                                   // 10%
                                                                                                   // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ControllerSchemeIO driver = new TwoStickDriveXboxOp(0, 1, 2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Network Tables
    // private NTSubsystem networkTables = new NTSubsystem(new Pose2d(), new Pose2d());

    // Vision
    // * Quest
    // private QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, networkTables);

    // Subsystems
    // * Shooter
    // private ClimberSubsystem climber = new ClimberSubsystem();
    // private IntakeSubsystem intake = new IntakeSubsystem();
    // private ManipulationSubsystem manipulation = new ManipulationSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();

    // subsytems var, contains all subsytems, less to implemnt into classes
//     private subsystems subsystems = new subsystems(drivetrain, questNav, shooter, climber, intake, manipulation);
//     private AutoDirector auto = new AutoDirector(subsystems);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driver.DriveLeft() * MaxSpeed)
                        .withVelocityY(-driver.DriveUp() * MaxSpeed)
                        .withRotationalRate(-driver.DriveTheta() * MaxAngularRate)));

        driveFacingAngle.HeadingController.setPID(Constants.Drivetrain.HeadingController.kP,
                Constants.Drivetrain.HeadingController.kI, Constants.Drivetrain.HeadingController.kD);

        // // Auto Shoot
        // driver.Shoot()
        //         .whileTrue(shooter.commands
        //                 .autoShoot(shooter
        //                         .getShootingData(drivetrain.getState().Pose.getX(),
        //                                 drivetrain.getState().Pose.getY(),
        //                                 drivetrain.getState().Speeds.vxMetersPerSecond,
        //                                 drivetrain.getState().Speeds.vyMetersPerSecond)
        //                         .shooterVelocity()));
        // // Shoot with constant values
        driver.Shoot()
                .whileTrue(shooter.commands
                        .autoShoot(Constants.ShooterConstants.ShooterMotorConfigs.leftSpeed,
                                Constants.ShooterConstants.ShooterMotorConfigs.rightSpeed)
                        .alongWith(shooter.commands
                                .autoAngle(Constants.ShooterConstants.HoodMotorConfigs.minAngle)));
        driver.Shoot().onFalse(shooter.commands.autoShoot(RotationsPerSecond.of(0)));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.brake().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.brake().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-driver.InputUp(), -driver.InputLeft()))));

        // driver.Intake().onTrue(intake.commands.intake().alongWith(manipulation.commands.intakeCommand()));
        // driver.Outtake().onTrue(intake.commands.outtake().alongWith(manipulation.commands.outtakeCommand()));
        // driver.Intake().and(driver.Outtake())
        //         .onFalse(intake.commands.idle().alongWith(manipulation.commands.idleCommand()));

        // driver.Intake().onTrue(manipulation.commands.intakeCommand());
        // driver.Outtake().onTrue(manipulation.commands.outtakeCommand());
        // driver.Intake().and(driver.Outtake())
        //         .onFalse(manipulation.commands.idleCommand());

        // Reset the field-centric heading on left bumper press.
        driver.seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Climber
        // driver.autoRetractClimber().onTrue(climber.commands.autoRetract());
        // driver.autoExtendClimber().onTrue(climber.commands.autoExtend());
        // driver.manExtendClimber().onTrue(climber.commands.manualExtend());
        // driver.manExtendClimber().onFalse(climber.commands.Stop());
        // driver.manRetractClimber().onTrue(climber.commands.manualRetract());
        // driver.manRetractClimber().onFalse(climber.commands.Stop());
        // driver.resetClimberEncoder().onTrue(climber.commands.resetEncoder());
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

    public void visionPeriodic() {
        // questNav.visionPeriodic();
    }

    public Command seed() {
        return Commands.runOnce(() -> {
            drivetrain.seedFieldCentric();
            // questNav.resetPose2D(questNav.getQuestPose2D().rotateBy(questNav.getQuestPose2D().getRotation().unaryMinus()));
        });
    }
}