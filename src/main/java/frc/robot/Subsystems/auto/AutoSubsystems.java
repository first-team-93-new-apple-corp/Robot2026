package frc.robot.Subsystems.auto;

import frc.robot.ShooterMath;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.QuestNav;

public record AutoSubsystems(
    CommandSwerveDrivetrain drivetrain, 
    QuestNav questNav,
    ShooterMath shooterMath
) {}