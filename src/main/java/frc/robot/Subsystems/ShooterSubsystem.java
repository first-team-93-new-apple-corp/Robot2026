package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterMath;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterCommands commands;

    public ShooterSubsystem(){
        commands = new ShooterCommands();
    }
    


    public Rotation2d getAlignedShooterR2D(double poseX, double poseY, double velX, double velY){
        return ShooterMath.generateRotation2d(poseX, poseY, velX, velY);
    }

    public class ShooterCommands{

    }
}
