package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public record ShootingData(
    Rotation2d drivetrainAngle,
    AngularVelocity shooterVelocity,
    Distance distance
) {} 
