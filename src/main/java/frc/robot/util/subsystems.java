package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public record subsystems(
    CommandSwerveDrivetrain drivetrain, 
    VisionSubsystem questNav,
    ShooterSubsystem shooter,
    ClimberSubsystem climber,
    IntakeSubsystem intake,
    ManipulationSubsystem manipulation
) {
    public subsystems(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, ClimberSubsystem climber, IntakeSubsystem intake, ManipulationSubsystem manipulation) {
        this(drivetrain, null, shooter, climber, intake, manipulation);
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

    public Command shoot(double secondsBeforeWiggle) {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(manipulation.commands.shootCommand());
        // return cmds.alongWith(Commands.waitSeconds(secondsBeforeWiggle)).andThen(intake.commands.wigglePivot(new Trigger(()->Commands.waitSeconds(5).isFinished())));
        return cmds;
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
        cmds.addCommands(shooter.commands.testingHood()); // Comment out after testing
        cmds.addCommands(shooter.commands.testingShooter()); // Comment out after testing
        return cmds;
     }

     public Command PrimeFalse(){
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter.commands.stopShooter());
        cmds.addCommands(shooter.commands.autoAngleNoOffset(Degrees.of(0)));
        return cmds;
     }

     public Command Outake(){
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands((intake.commands.outtake()));
        cmds.addCommands(manipulation.commands.outtakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
     }

     public Command OutakeFalse(){
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        // var cmd1 = intake.commands.autoPivotDown();
        cmds.addCommands((intake.commands.idle()));
        cmds.addCommands(manipulation.commands.idleCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(0)));
        return cmds;
     }
     
}