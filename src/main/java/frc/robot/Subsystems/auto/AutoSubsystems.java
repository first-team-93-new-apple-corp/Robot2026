package frc.robot.Subsystems.auto;

import frc.robot.ShooterMath;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.QuestNavSubsystem;

public record AutoSubsystems(
    CommandSwerveDrivetrain drivetrain, 
    QuestNavSubsystem questNav,
    ShooterMath shooterMath
) {}