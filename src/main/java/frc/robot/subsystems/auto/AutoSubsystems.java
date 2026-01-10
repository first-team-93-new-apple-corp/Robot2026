package frc.robot.subsystems.auto;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;

public record AutoSubsystems(
    CommandSwerveDrivetrain drivetrain, 
    QuestNavSubsystem questNav
) {}