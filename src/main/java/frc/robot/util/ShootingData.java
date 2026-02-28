package frc.robot.util;

import static  edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

public record ShootingData(
    Rotation2d drivetrainAngle,
    Angle shooterAngle,
    AngularVelocity shooterVelocity
    
) {
} 