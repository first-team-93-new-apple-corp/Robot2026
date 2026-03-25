// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Controls.ControllerSchemeIO;
import frc.robot.Controls.TwoStickDriveXboxOp;
// import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.PowerDistributionSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.auto.AutoDirector;
import frc.robot.generated.TunerConstants;
import frc.robot.util.NTSubsystem;
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

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ControllerSchemeIO driver = new TwoStickDriveXboxOp(0, 1, 2);
    // private final ControllerSchemeIO driver = new XboxDrive(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Network Tables
    private NTSubsystem networkTables = new NTSubsystem();

    // Vision
    // * Quest
    private VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain, networkTables);

    // Subsystems
    // * Shooter
    // private ClimberSubsystem climber = new ClimberSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ManipulationSubsystem manipulation = new ManipulationSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    public final PowerDistributionSubsystem DistributionHubsystem = new PowerDistributionSubsystem();

    // subsytems var, contains all subsytems, less to implemnt into classes
    private subsystems subsystems = new subsystems(drivetrain, visionSubsystem, shooter, intake, manipulation, driver);
    private AutoDirector auto = new AutoDirector(subsystems);

    

    public RobotContainer() {
        RobotController.setBrownoutVoltage(Volts.of(6.5));

        configureBindings();

        

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(driver.DriveLeft())
                        .withVelocityY(driver.DriveUp())
                        .withRotationalRate(driver.DriveTheta())));

        driver.robotRel()
                .whileTrue(drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(driver.DriveLeft())
                                .withVelocityY(driver.DriveUp())
                                .withRotationalRate(driver.DriveTheta())));

        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //         drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.brake().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.brake().whileTrue(drivetrain
        //         .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.InputUp(), -driver.InputLeft()))));

        // Reset the field-centric heading on left bumper press.
        driver.seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        driver.Shoot().whileTrue(subsystems.Prime().andThen(subsystems.shoot()));
        driver.Shoot().onFalse(subsystems.shootFalse());

        driver.Prime().whileTrue(subsystems.Prime().repeatedly());
        driver.Prime().onFalse(subsystems.PrimeFalse());

        driver.DriverPrime().whileTrue(subsystems.DriverPrime().repeatedly());

        driver.PrimeClose().whileTrue(subsystems.PrimeHubClose());
        driver.PrimeFar().whileTrue(subsystems.PrimeHubFar());
        driver.PrimeLeft().whileTrue(subsystems.PrimeHubLeft());
        driver.PrimeRight().whileTrue(subsystems.PrimeHubRight());

        driver.PrimeClose().onFalse(subsystems.PrimeFalse());
        driver.PrimeFar().onFalse(subsystems.PrimeFalse());
        driver.PrimeLeft().onFalse(subsystems.PrimeFalse());
        driver.PrimeRight().onFalse(subsystems.PrimeFalse());

        driver.LowerIntake().onTrue(subsystems.intake().commands.autoPivotDown());
        driver.RaiseIntake().onTrue(subsystems.intake().commands.autoPivotUp());
        driver.middleIntake().onTrue(subsystems.intake().commands.autoPivotMiddle());

        driver.WiggleIntake().whileTrue(subsystems.intake().commands.wigglePivot(driver.WiggleIntake()));

        driver.Intake().onTrue(subsystems.Intake());
        driver.Intake().onFalse(subsystems.IntakeFalse());

        driver.Outtake().onTrue(subsystems.Outake());
        driver.Outtake().onFalse(subsystems.OutakeFalse());
        driver.Pass().onTrue(subsystems.pass());
        driver.Pass().onFalse(subsystems.PrimeFalse());

        driver.manShoot().whileTrue(subsystems.shooter().commands.autoShoot(()-> RotationsPerSecond.of((driver.leftTrigger())).times(100)).repeatedly());
        driver.manShoot().onFalse(subsystems.shooter().commands.stopShooter());
        driver.manHood().whileTrue(subsystems.shooter().commands.autoAngleNoOffset(()-> Degrees.of(driver.rightTrigger()).times(18)).repeatedly());
        driver.manHood().onFalse(subsystems.shooter().commands.autoAngleNoOffset(Degrees.of(0)));

        // driver.testingButton().onTrue(AutoBuilder.pathfindToPose(new Pose2d(subsystems.drivetrain().getState().Pose.getX()-1,subsystems.drivetrain().getState().Pose.getY(), subsystems.drivetrain().getState().Pose.getRotation()) , AutoConstants.constraints));
        RobotModeTriggers.autonomous().onTrue(subsystems.questNav().commands.resetPose().ignoringDisable(true));
        RobotModeTriggers.teleop().onTrue(subsystems.questNav().commands.resetPose().ignoringDisable(true));
        driver.resetPose().onTrue(subsystems.questNav().commands.resetPose().ignoringDisable(true).andThen(Commands.print("Reset Pose due to Button Press")));
        
        // driver.autoExtendClimber().onTrue(subsystems.climber().commands.autoExtend());
        // driver.autoRetractClimber()
        //         .onTrue(subsystems.climber().commands.manualRetract()
        //                 .andThen(Commands.waitUntil(() -> subsystems.climber().isAtBottom()))
        //                 .andThen(subsystems.climber().commands.Stop()));

        // RobotModeTriggers.test()
        //         .onTrue(subsystems.climber().commands.manualRetract()
        //                 .andThen(Commands.waitUntil(() -> subsystems.climber().isAtBottom()))
        //                 .andThen(subsystems.climber().commands.Stop()));
    }

    public Command getAutonomousCommand() {
        return auto.autoChooser.getSelected().command();
    }

    public void visionPeriodic() {
        visionSubsystem.visionPeriodic();
    }

    public void telePeriodic() {
        double[] test = { subsystems.getShootingData().drivetrainAngle().getDegrees(),  //drivetrain, speed, angle
                subsystems.getShootingData().shooterVelocity().in(RotationsPerSecond),
                (subsystems.getShootingData().shooterAngle()).in(Degrees) };
        SmartDashboard.putNumberArray("Target Shooting Math", test);
    }

    public Command seed() {
        return Commands.runOnce(() -> {
            drivetrain.seedFieldCentric();
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