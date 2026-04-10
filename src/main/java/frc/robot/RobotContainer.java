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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls.ControllerSchemeIO;
import frc.robot.Controls.TwoStickDriveXboxOp;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.PowerDistributionSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.auto.AutoDirector;
import frc.robot.util.Elastic;
import frc.robot.util.NTSubsystem;
import frc.robot.util.ShootingData;
import frc.robot.util.subsystems;
import dev.doglog.*;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static double RPM_Tuning = 20.0;
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        // private final Telemetry logger = new Telemetry(MaxSpeed);

        private final ControllerSchemeIO driver = new TwoStickDriveXboxOp(0, 1, 2);
        // private final ControllerSchemeIO driver = new XboxDrive(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Network Tables
        private NTSubsystem networkTables = new NTSubsystem();

        // Vision
        // * Quest
        private VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain, networkTables);

        // Subsystems
        private IntakeSubsystem intake = new IntakeSubsystem();
        private ManipulationSubsystem manipulation = new ManipulationSubsystem();
        private ShooterSubsystem shooter = new ShooterSubsystem();
        public PowerDistributionSubsystem DistributionHubsystem = new PowerDistributionSubsystem();

        // subsytems var, contains all subsytems, less to implemnt into classes
        public subsystems subsystems = new subsystems(drivetrain, visionSubsystem, shooter, intake, manipulation,
                        DistributionHubsystem, driver);
        private AutoDirector auto = new AutoDirector(subsystems, networkTables);

        public RobotContainer() {
                RobotController.setBrownoutVoltage(Volts.of(6.5));
                configureBindings();
                DogLog.setOptions(new DogLogOptions()
                                .withCaptureDs(true)
                                .withLogExtras(false)
                                .withCaptureConsole(true)
                                .withCaptureNt(false)
                                .withUseLogThread(true));
                DogLog.setEnabled(true);
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(driver.DriveLeft())
                                                .withVelocityY(driver.DriveUp())
                                                .withRotationalRate(driver.DriveTheta())));

                driver.robotRel()
                                .whileTrue(drivetrain
                                                .applyRequest(() -> robotCentricDrive.withVelocityX(driver.DriveLeft())
                                                                .withVelocityY(driver.DriveUp())
                                                                .withRotationalRate(driver.DriveTheta())));

                // final var idle = new SwerveRequest.Idle();
                // RobotModeTriggers.disabled().whileTrue(
                // drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.brake().whileTrue(drivetrain.applyRequest(() -> brake));
                // driver.brake().whileTrue(drivetrain
                // .applyRequest(() -> point.withModuleDirection(new
                // Rotation2d(-driver.InputUp(), -driver.InputLeft()))));

                // Reset the field-centric heading on left bumper press.
                driver.seed().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
                // drivetrain.registerTelemetry(logger::telemeterize);

                driver.Shoot().whileTrue(subsystems.shoot());
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

                driver.manDownIntake().onTrue(subsystems.intake().commands.manPivotDown());
                driver.manUpIntake().onTrue(subsystems.intake().commands.manPivotUp());
                driver.manDownIntake().onFalse(subsystems.intake().commands.manPivotStop());
                driver.manUpIntake().onFalse(subsystems.intake().commands.manPivotStop());

                driver.WiggleIntake().whileTrue(subsystems.intake().commands.wigglePivot(driver.WiggleIntake()));
                driver.WiggleIntake().onFalse(subsystems.manipulation().commands.idleCommand());

                driver.Intake().onTrue(subsystems.Intake());
                driver.Intake().onFalse(subsystems.IntakeFalse());

                driver.Outtake().onTrue(subsystems.Outake());
                driver.Outtake().onFalse(subsystems.OutakeFalse());
                driver.Pass().onTrue(subsystems.pass());
                driver.Pass().onFalse(subsystems.PrimeFalse());

                driver.manShoot().whileTrue(subsystems.shooter().commands
                                .autoShoot(() -> RotationsPerSecond.of((driver.leftTrigger())).times(100))
                                .repeatedly());
                driver.manShoot().onFalse(subsystems.shooter().commands.stopShooter());
                driver.manHood().whileTrue(subsystems.shooter().commands
                                .autoAngleNoOffset(() -> Degrees.of(driver.rightTrigger()).times(18)).repeatedly());
                driver.manHood().onFalse(subsystems.shooter().commands.autoAngleNoOffset(Degrees.of(0)));

                // RobotModeTriggers.autonomous().onTrue(subsystems.vision().commands.resetPose().ignoringDisable(true));
                RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> Elastic.selectTab(1)).ignoringDisable(true));
                RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> Elastic.selectTab(0)).ignoringDisable(true));
                // RobotModeTriggers.teleop().onTrue(subsystems.vision().commands.resetPose().ignoringDisable(true));
                (new Trigger(() -> auto.hasSelectedAutoPreviewChanged()).and(RobotModeTriggers.disabled()))
                                .onTrue(Commands.runOnce(auto::updateSelectedAutoPreview).ignoringDisable(true));
                RobotModeTriggers.teleop().onTrue(Commands.runOnce(auto::removePreview));

                driver.resetPose().onTrue(subsystems.vision().commands.resetPose().ignoringDisable(true)
                                .andThen(Commands.print("Reset Pose due to Button Press")));

                // driver.SysIdForwardQ().whileTrue(subsystems.drivetrain().commands.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // driver.SysIdForwardD().whileTrue(subsystems.drivetrain().commands.sysIdDynamic(SysIdRoutine.Direction.kForward));
                // driver.SysIdBackwordQ().whileTrue(subsystems.drivetrain().commands.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                // driver.SysIdBackwardD().whileTrue(subsystems.drivetrain().commands.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                // // driver.SysIdBackwardD().or(null)
                // driver.SysIdStart().onTrue(Commands.runOnce(SignalLogger::start));
                // driver.SysIdEnd().onTrue(Commands.runOnce(SignalLogger::stop));
                // driver.SysIdCCW().onTrue(subsystems.drivetrain().commands.sysIdDynamic(SysIdRoutine.Direction.kForward));

        }

    public Command getAutonomousCommand() {
        return auto.autoChooser.getSelected().command();
    }

    

    public  void telePeriodic() {
        double[] test = { subsystems.getShootingData().drivetrainAngle().getDegrees(), // drivetrain, speed, angle
                subsystems.getShootingData().shooterVelocity().in(RotationsPerSecond),
                (subsystems.getShootingData().shooterAngle()).in(Degrees) };
        double[] test2 = { subsystems.getShootingDataFallback().drivetrainAngle().getDegrees(),  //drivetrain, speed, angle
                subsystems.getShootingDataFallback().shooterVelocity().in(RotationsPerSecond),
                (subsystems.getShootingDataFallback().shooterAngle()).in(Degrees),
            subsystems.getShootingDataFallback().distance().in(Meters)};
        SmartDashboard.putNumberArray("Target Shooting Math", test);
        SmartDashboard.putNumber("Target Fallback Math", RobotContainer.RPM_Tuning);
        SmartDashboard.putNumber("Distance", subsystems.getShootingDataFallback().distance().magnitude());
        RPM_Tuning = SmartDashboard.getNumber("Tuning",20);

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

        // private double getDriveSpeedX() {
        // return drivetrain.getState().Speeds.vxMetersPerSecond;
        // }

        // private double getDriveSpeedY() {
        // return drivetrain.getState().Speeds.vyMetersPerSecond;
        // }

        // public ShootingData getShootingData() {
        // return shooter.getShootingData(getDrivePoseX(), getDrivePoseY(),
        // getDriveSpeedX(),
        // getDriveSpeedY());
        // }
        public ShootingData getShootingDataFallback() {
                return shooter.getShootingDataFallback(getDrivePoseX(), getDrivePoseY());
        }
}
