package frc.robot.util;

import frc.robot.ShooterMath;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.QuestNavSubsystem;

public record subsystems(
    CommandSwerveDrivetrain drivetrain, 
    QuestNavSubsystem questNav,
    ShooterMath shooterMath
) {}