package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants.ShooterConstants.Presets;
import frc.robot.RobotContainer;
import frc.robot.Controls.ControllerSchemeIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.PowerDistributionSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.auto.AutoConstants;
import frc.robot.Subsystems.ShooterSubsystem;
import dev.doglog.*;

public record subsystems(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        ShooterSubsystem shooter,
        IntakeSubsystem intake,
        ManipulationSubsystem manipulation,
        PowerDistributionSubsystem pds,
        ControllerSchemeIO driver) {
    public subsystems(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
            IntakeSubsystem intake, ManipulationSubsystem manipulation, PowerDistributionSubsystem pds, ControllerSchemeIO driver) {
        this(drivetrain, null, shooter, intake, manipulation, pds, driver);
    }

    public Command Intake() {
        DogLog.timestamp("Intake");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands(cmd1.andThen(intake.commands.intake()));
        cmds.addCommands(manipulation.commands.intakeCommand());
        // cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command intakeDepot() {
        DogLog.timestamp("intakeDepot");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.intake());
        cmds.addCommands(manipulation.commands.intakeCommand());
        // cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command IntakeFalse() {
        DogLog.timestamp("IntakeFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        // cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
    }

    public Command shoot() {
        DogLog.timestamp("shoot");
        return manipulation.commands.shootCommand();
    }

    public Command pass() {
        DogLog.timestamp("pass");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> Degrees.of(42), () -> RotationsPerSecond.of(100)));
        return cmds.withTimeout(0.5);
    }

    public Command shootFalse() {
        DogLog.timestamp("shootFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        // cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
    }

    // public Command Prime() {
    // DogLog.timestamp("Prime");
    // ParallelCommandGroup cmds = new ParallelCommandGroup();
    // cmds.addCommands((intake.commands.idle()));
    // cmds.addCommands(shooter().commands.velocityAndHood(() ->
    // getShootingData().shooterAngle(), () ->
    // getShootingData().shooterVelocity()));
    // // cmds.addCommands(shooter().commands.velocityAndHood(() ->
    // getShootingData().shooterAngle(), () ->
    // getShootingData().shooterVelocity().times(efficiencyCalculate())));
    // return cmds.withTimeout(1);
    // }
    public Command Prime() {
        DogLog.timestamp("Prime");
        ParallelCommandGroup cmds = new ParallelCommandGroup();

        cmds.addCommands((intake.commands.idle()));
        
        // cmds.addCommands(drivetrain.applyRequest(
        //         () -> drivetrain().driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle())
        //                 .withVelocityX(driver.DriveLeft()).withVelocityY(driver.DriveUp())));
        
        // cmds.addCommands(shooter().commands.velocityAndHood(() -> getShootingDataFallback().shooterAngle(),
        //         () -> RotationsPerSecond.of(RobotContainer.RPM_Tuning)));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> getShootingDataFallback().shooterAngle(),
                () -> getShootingDataFallback().shooterVelocity()));

       
        return cmds.withTimeout(1);
    }

    public Command DriverPrime() {
        DogLog.timestamp("DriverPrime");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(drivetrain.applyRequest(
                () -> drivetrain().driveFacingAngle.withTargetDirection(getAngleToHub())
                        .withVelocityX(driver.DriveLeft()).withVelocityY(driver.DriveUp())));
        return cmds.withTimeout(1);
    }

    public Command PrimeHubClose() {
        DogLog.timestamp("PrimeHubClose");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> Presets.close));
        return cmds.withTimeout(1);
    }

    public Command PrimeHubCloseSide() {
        DogLog.timestamp("PrimeHubCloseSide");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> Presets.closeSide));
        return cmds.withTimeout(1);
    }

    public Command PrimeHubFar() {
        DogLog.timestamp("PrimeHubFar");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> Presets.inFrontOfClimb));
        return cmds.withTimeout(1);
    }

    public Command PrimeHubLeft() {
        DogLog.timestamp("PrimeHubLeft");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> Presets.closeSide));
        return cmds.withTimeout(1);
    }

    public Command PrimeHubRight() {
        DogLog.timestamp("PrimeHubRight");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter().commands.velocityAndHood(() -> Presets.closeSide));
        return cmds.withTimeout(1);
    }

    // public double efficiencyCalculate() {
    // double hubX = AutoConstants.Hub.getHub().getX();
    // double hubY = AutoConstants.Hub.getHub().getY();
    // double distance = Math.sqrt(Math.pow(hubX -
    // drivetrain.getState().Pose.getX(), 2)
    // + Math.pow(hubY - drivetrain.getState().Pose.getY(), 2));
    // double a = 0.0444713;
    // double b = -0.474115;
    // double c = 1.90894;
    // double d = -0.803343;
    // return a * (Math.pow(distance, 3)) + b * (Math.pow(distance, 2)) + c *
    // distance + d;
    // }

    // public Command AutoPrime() {
    // DogLog.timestamp("AutoPrime");
    // ParallelCommandGroup cmds = new ParallelCommandGroup();
    // cmds.addCommands((intake.commands.idle()));
    // cmds.addCommands(drivetrain.applyRequest(
    // () ->
    // drivetrain().driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle())));
    // cmds.addCommands(shooter().commands.velocityAndHood(() ->
    // getShootingData().shooterAngle(),
    // () -> getShootingData().shooterVelocity()));
    // return cmds.withTimeout(2);
    // }
    public Command AutoPrime() {
        DogLog.timestamp("AutoPrime");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        // cmds.addCommands(drivetrain.applyRequest(
        // () ->
        // drivetrain().driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle())));
        // cmds.addCommands(shooter().commands.velocityAndHood(() ->
        // getShootingData().shooterAngle(),
        // () -> getShootingData().shooterVelocity()));
        return cmds.withTimeout(2);
    }

    public Command PrimeFalse() {
        DogLog.timestamp("PrimeFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(shooter.commands.stopShooter());
        cmds.addCommands(shooter.commands.autoAngleNoOffset(Degrees.of(0)));
        return cmds;
    }

    public Command Outake() {
        DogLog.timestamp("Outake");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands((intake.commands.outtake()));
        cmds.addCommands(manipulation.commands.outtakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command OutakeFalse() {
        DogLog.timestamp("OutakeFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(manipulation.commands.idleCommand());
        cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
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
    public ShootingData getShootingDataFallback() {
                return shooter.getShootingDataFallback(getDrivePoseX(), getDrivePoseY());
        }

    public Rotation2d getAngleToHub() {
        return new Rotation2d(Radians.of(shooter.getAngleToHub()));
    }

}