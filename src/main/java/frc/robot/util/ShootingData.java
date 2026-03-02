package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record ShootingData(
    Rotation2d drivetrainAngle,
    Angle shooterAngle,
    AngularVelocity shooterVelocity
) {} 
