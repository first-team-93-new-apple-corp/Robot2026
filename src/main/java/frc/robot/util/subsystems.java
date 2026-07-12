package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants.ShooterConstants.Presets;
import frc.robot.Controls.ControllerSchemeIO;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.PowerDistributionSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
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
            IntakeSubsystem intake, ManipulationSubsystem manipulation, PowerDistributionSubsystem pds,
            ControllerSchemeIO driver) {
        this(drivetrain, null, shooter, intake, manipulation, pds, driver);
    }

    public Command Intake() {
        DogLog.timestamp("Intake");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands(cmd1.andThen(intake.commands.intake()));
        cmds.addCommands(manipulation.commands.intakeCommand());
        return cmds;
    }

    public Command intakeDepot() {
        DogLog.timestamp("intakeDepot");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.intake());
        cmds.addCommands(manipulation.commands.intakeCommand());
        return cmds;
    }

    public Command IntakeFalse() {
        DogLog.timestamp("IntakeFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        return cmds;
    }

    public Command shoot() {
        DogLog.timestamp("shoot");
        return manipulation.commands.shootCommand();
    }

    public Command pass() {
        DogLog.timestamp("pass");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoShoot(() -> RotationsPerSecond.of(100)));
        return cmds;
    }

    public Command shootFalse() {
        DogLog.timestamp("shootFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(manipulation.commands.idleCommand());
        return cmds;
    }

    
    public Command Prime() {
        DogLog.timestamp("Prime");
        ParallelCommandGroup cmds = new ParallelCommandGroup();

        cmds.addCommands((intake.commands.idle()));


        // We need below for tuning 
        // cmds.addCommands(shooter().commands.autoShoot(() -> RotationsPerSecond.of(RobotContainer.RPM_Tuning)));

        // Below is shooter math implmentation
        cmds.addCommands(shooter().commands.autoShoot(() -> getShootingDataFallback().shooterVelocity()));

        return cmds;
    }

    public Command DriverPrime() {
        DogLog.timestamp("DriverPrime");
        ParallelCommandGroup cmds = new ParallelCommandGroup();

        cmds.addCommands(drivetrain.applyRequest(
                () -> drivetrain().driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle())
                        .withVelocityX(driver.DriveLeft()).withVelocityY(driver.DriveUp())));

        return cmds;
    }

    public Command PrimeHubClose() {
        DogLog.timestamp("PrimeHubClose");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoShoot(() -> Presets.close));
        return cmds;
    }

    public Command PrimeHubCloseSide() {
        DogLog.timestamp("PrimeHubCloseSide");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoShoot(() -> Presets.closeSide));
        return cmds;
    }

    public Command PrimeHubFar() {
        DogLog.timestamp("PrimeHubFar");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoShoot(() -> Presets.inFrontOfClimb));
        return cmds;
    }

    public Command PrimeHubLeft() {
        DogLog.timestamp("PrimeHubLeft");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoShoot(() -> Presets.closeSide));
        return cmds;
    }

    public Command PrimeHubRight() {
        DogLog.timestamp("PrimeHubRight");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter().commands.autoShoot(() -> Presets.closeSide));
        return cmds;
    }

   
    public Command AutoPrime() {
        DogLog.timestamp("AutoPrime");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        
        return cmds;
    }

    public Command PrimeFalse() {
        DogLog.timestamp("PrimeFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
    }

    public Command Outake() {
        DogLog.timestamp("Outake");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands((intake.commands.outtake()));
        cmds.addCommands(manipulation.commands.outtakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command OutakeFalse() {
        DogLog.timestamp("OutakeFalse");
        ParallelCommandGroup cmds = new ParallelCommandGroup();
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
    // orthogonal  hub
    // private double getDriveSpeedY2() {
    //     SwerveDriveState state = drivetrain.getState();
    //     SmartDashboard.putString("State", state.Speeds.vxMetersPerSecond+ " " +  state.Speeds.vyMetersPerSecond);

    //     Translation2d fieldVelocity = new Translation2d(state.Speeds.vyMetersPerSecond,
    //             state.Speeds.vxMetersPerSecond)
    //             .rotateBy(state.Pose.getRotation().unaryMinus());
        

                
    //     Translation2d hubVelocity = fieldVelocity
    //             .rotateBy(new Rotation2d(Radians.of(Math.PI/2).minus(angleToAlign(state.Pose.getX(), AutoConstants.Hub.getHub().getX(),state.Pose.getY(),AutoConstants.Hub.getHub().getY()))).unaryMinus());
        
    //     return hubVelocity.getY();

    // }
    // towards hub
    // private double getDriveSpeedX2() {
    //     SwerveDriveState state = drivetrain.getState();
    //     SmartDashboard.putString("State", state.Speeds.vxMetersPerSecond+ " " +  state.Speeds.vyMetersPerSecond);

    //     Translation2d fieldVelocity = new Translation2d(state.Speeds.vyMetersPerSecond,
    //             state.Speeds.vxMetersPerSecond)
    //             .rotateBy(state.Pose.getRotation().unaryMinus());
    //     SmartDashboard.putNumber("Field Velocity (x)", fieldVelocity.getX());
    //     SmartDashboard.putNumber("Field Velocity (y)", fieldVelocity.getY());

                
    //     Translation2d hubVelocity = fieldVelocity
    //             .rotateBy(new Rotation2d(Radians.of(Math.PI/2).minus(angleToAlign(state.Pose.getX(), AutoConstants.Hub.getHub().getX(),state.Pose.getY(),AutoConstants.Hub.getHub().getY()))).unaryMinus());
    //     SmartDashboard.putNumber("Align angle",angleToAlign(state.Pose.getX(), AutoConstants.Hub.getHub().getX(),state.Pose.getY(),AutoConstants.Hub.getHub().getY()).in(Degrees) );
    //      SmartDashboard.putNumber("Hub Velocity (x)", hubVelocity.getX());
    //     SmartDashboard.putNumber("Hub Velocity (y)", hubVelocity.getY());
    //     return hubVelocity.getX();

    // }

    public ShootingData getShootingData() {
        return shooter.getShootingData(getDrivePoseX(), getDrivePoseY(), getDriveSpeedX(),
                getDriveSpeedY());
    }

    public ShootingData getShootingDataFallback() {
        return shooter.getShootingDataFallback(getDrivePoseX(), getDrivePoseY());
    }

    public static Angle angleToAlign(double robotX, double hubX, double robotY, double hubY) {
        double angleToHub = 0;
        angleToHub = Math.atan2((hubY - robotY), (hubX - robotX));

        return Radians.of(angleToHub);
    }

}