package frc.robot.util;

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
) {}