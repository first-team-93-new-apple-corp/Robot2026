package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.Controls.ControllerSchemeIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.auto.AutoConstants;
import frc.robot.Subsystems.ShooterSubsystem;

public record subsystems(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem questNav,
        ShooterSubsystem shooter,
        ClimberSubsystem climber,
        IntakeSubsystem intake,
        ManipulationSubsystem manipulation,
        ControllerSchemeIO driver) {
    public subsystems(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, ClimberSubsystem climber,
            IntakeSubsystem intake, ManipulationSubsystem manipulation, ControllerSchemeIO driver) {
        this(drivetrain, null, shooter, climber, intake, manipulation, driver);
    }

    public Command Intake() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands(cmd1.andThen(intake.commands.intake()));
        cmds.addCommands(manipulation.commands.intakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command intakeDepot() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.intake());
        cmds.addCommands(manipulation.commands.intakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command IntakeFalse() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
    }

    public Command shoot() {
        return manipulation.commands.shootCommand();
    }

    public Command shootFalse() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        // cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
    }

    public Command Prime() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(drivetrain.applyRequest(
                () -> drivetrain().driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle())
                        .withVelocityX(driver.DriveLeft()).withVelocityY(driver.DriveUp())));
        cmds.addCommands(shooter().commands.autoAngle(() -> getShootingData().shooterAngle())
                .andThen(shooter().commands.autoShoot(() -> getShootingData().shooterVelocity())));
        return cmds.withTimeout(2);
    }

    public Command PrimeHubClose() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoAngle(() -> AutoConstants.PresetShootingPoints.getClose().hoodAngle())
                .andThen(shooter().commands.autoShoot(() -> AutoConstants.PresetShootingPoints.getClose().velocity().times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierClose))));
        return cmds.withTimeout(2);
    }

    public Command PrimeHubFar() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoAngleNoOffset(() -> AutoConstants.PresetShootingPoints.getFar().hoodAngle())
                .andThen(shooter().commands.autoShoot(() -> AutoConstants.PresetShootingPoints.getFar().velocity().times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierFar))));
        return cmds.withTimeout(2);
    }

    public Command PrimeHubLeft() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoAngleNoOffset(() -> AutoConstants.PresetShootingPoints.getLeft().hoodAngle())
                .andThen(shooter().commands.autoShoot(() -> AutoConstants.PresetShootingPoints.getLeft().velocity().times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierFar))));
        return cmds.withTimeout(2);
    }
    public Command PrimeHubRight() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoAngleNoOffset(() -> AutoConstants.PresetShootingPoints.getRight().hoodAngle())
                .andThen(shooter().commands.autoShoot(() -> AutoConstants.PresetShootingPoints.getRight().velocity().times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierFar))));
        return cmds.withTimeout(2);
    }

    public Command AutoPrime() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(drivetrain.applyRequest(
                () -> drivetrain().driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle())));
        cmds.addCommands(shooter().commands.autoAngle(() -> getShootingData().shooterAngle())
                .andThen(shooter().commands.autoShoot(() -> getShootingData().shooterVelocity())));
        return cmds.withTimeout(2);
    }

    public Command PrimeFalse() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter.commands.stopShooter());
        cmds.addCommands(shooter.commands.autoAngleNoOffset(Degrees.of(0)));
        return cmds;
    }

    public Command Outake() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands((intake.commands.outtake()));
        cmds.addCommands(manipulation.commands.outtakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command OutakeFalse() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(manipulation.commands.idleCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(0)));
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

}