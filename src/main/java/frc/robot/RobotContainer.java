// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Controls.ControllerSchemeIO;
import frc.robot.Controls.TwoStickDriveXboxOp;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.PowerDistributionSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ShootingData;
import frc.robot.util.subsystems;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(constants.Swerve.MaxSpeed * constants.Controls.Deadzone)
            .withRotationalDeadband(constants.Swerve.MaxAngularRate * constants.Controls.Deadzone) // Add a
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
    private ClimberSubsystem climber = new ClimberSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ManipulationSubsystem manipulation = new ManipulationSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
     public final PowerDistributionSubsystem DistributionHubsystem = new PowerDistributionSubsystem();

    // subsytems var, contains all subsytems, less to implemnt into classes
    private subsystems subsystems = new subsystems(drivetrain, shooter, climber, intake, manipulation);
//     private AutoDirector auto = new AutoDirector(subsystems);

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

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.brake().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.brake().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.InputUp(), -driver.InputLeft()))));

        // driver.Intake().onTrue(m_ManipulationSubsystem.commands.intakeCommand());
        // driver.Outtake().onTrue(m_ManipulationSubsystem.commands.outtakeCommand());
        // driver.Intake().and(driver.Outtake()).onFalse(m_ManipulationSubsystem.commands.idleCommand());

        // Reset the field-centric heading on left bumper press.
        driver.seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        // driver.autoRetractClimber().onTrue(m_ClimberSubsystem.commands.autoRetract());

        // driver.autoExtendClimber().onTrue(m_ClimberSubsystem.commands.autoExtend());
        // driver.manExtendClimber().onTrue(m_ClimberSubsystem.commands.manualExtend());
        // driver.manExtendClimber().onFalse(m_ClimberSubsystem.commands.Stop());
        // driver.manRetractClimber().onTrue(m_ClimberSubsystem.commands.manualRetract());
        // driver.manRetractClimber().onFalse(m_ClimberSubsystem.commands.Stop());
        // driver.resetClimberEncoder().onTrue(m_ClimberSubsystem.commands.resetEncoder());
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

    private double getDrivePoseX() {
        return drivetrain.getState().Pose.getX();
    }

    private double getDrivePoseY() {
        return drivetrain.getState().Pose.getY();
    }

    private double getDriveSpeedX() {
        return drivetrain.getState().Speeds.vxMetersPerSecond;
    }

    private double getDriveSpeedY() {
        return drivetrain.getState().Speeds.vyMetersPerSecond;
    }

    public ShootingData getShootingData() {
        return shooter.getShootingData(getDrivePoseX(), getDrivePoseY(), getDriveSpeedX(),
                getDriveSpeedY());
    }
}