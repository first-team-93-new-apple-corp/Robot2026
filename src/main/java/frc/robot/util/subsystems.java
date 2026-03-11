package frc.robot.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ManipulationSubsystem;
import frc.robot.Subsystems.QuestNavSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public record subsystems(
    CommandSwerveDrivetrain drivetrain, 
    QuestNavSubsystem questNav,
    ShooterSubsystem shooter,
    ClimberSubsystem climber,
    IntakeSubsystem intake,
    ManipulationSubsystem manipulation
) {
    public subsystems(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, ClimberSubsystem climber, IntakeSubsystem intake, ManipulationSubsystem manipulation) {
        this(drivetrain, null, shooter, climber, intake, manipulation);
    }

    public Command intakeProtocol() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.intake());
        cmds.addCommands(manipulation.commands.intakeCommand());
        cmds.addCommands(shooter.commands.autoShoot(RotationsPerSecond.of(-0.5)));
        return cmds;
    }

    public Command intakeProtocolOnFalse() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
    }

    public Command shootProtocol() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(manipulation.commands.shootCommand());
        return cmds.alongWith(Commands.waitSeconds(4)).andThen(intake.commands.wigglePivot(new Trigger(()->Commands.waitSeconds(5).isFinished())));
    }

     public Command shootProtocolOnFalse() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(intake.commands.idle());
        cmds.addCommands(manipulation.commands.idleCommand());
        cmds.addCommands(shooter.commands.stopShooter());
        return cmds;
     }

     public Command primeProtocol() {
        ParallelCommandGroup cmds = new ParallelCommandGroup();
        cmds.addCommands(shooter.commands.testingHood()); // Comment out after testing
        cmds.addCommands(shooter.commands.testingShooter()); // Comment out after testing
        return cmds;
     }
}