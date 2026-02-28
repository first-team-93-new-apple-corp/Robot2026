package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingData;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterCommands commands;

    public ShootingData data;

    public ShooterSubsystem(){
        commands = new ShooterCommands();
    }
    


    public ShootingData getShootingData(double poseX, double poseY, double velX, double velY){
        data = ShooterMath.generateRotation2d(poseX, poseY, velX, velY);
        return data;
    }

    public ShootingData getShootingData(){
        return data;
    }



    public class ShooterCommands{
        public ShooterCommands(){

        }
    }
}
