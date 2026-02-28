package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.HoodMotorConfigs;
import frc.robot.Constants.ShooterConstants.ShooterMotorConfigs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingData;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterCommands commands;

    public ShootingData data;

    private TalonFX topLeftShooter;
    private TalonFX topRightShooter;
    private TalonFX bottomLeftShooter;
    private TalonFX bottomRightShooter;

    private TalonFX hoodMotor;
    
    private TalonFXConfiguration allShooterConfig;
    private TalonFXConfiguration hoodConfig;

    private StrictFollower follower;

    private Slot0Configs shooterSlot0Configs;

    private Slot0Configs hoodSlot0Configs;

    // private PIDController leftShooterPID = new PIDController(Constants.ShooterConstants.ShooterPID_P, Constants.ShooterConstants.ShooterPID_I, Constants.ShooterConstants.ShooterPID_D);

    public ShooterSubsystem(){
        commands = new ShooterCommands();
        topLeftShooter = new TalonFX(Constants.CAN.topLeftShooter);
        topRightShooter = new TalonFX(Constants.CAN.topRightShooter);
        bottomLeftShooter = new TalonFX(Constants.CAN.bottomLeftShooter);
        bottomRightShooter = new TalonFX(Constants.CAN.bottomRightShooter);

        allShooterConfig = new TalonFXConfiguration();

        allShooterConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterMotorConfigs.StatorLimitEnable;
        allShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterMotorConfigs.StatorLimit;
        allShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterMotorConfigs.SupplyLimitEnable;
        allShooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterMotorConfigs.SupplyLimit;

        shooterSlot0Configs = new Slot0Configs();

        shooterSlot0Configs.kS = ShooterMotorConfigs.kS;
        shooterSlot0Configs.kV = ShooterMotorConfigs.kV; 
        shooterSlot0Configs.kP = ShooterMotorConfigs.kP; 
        shooterSlot0Configs.kI = ShooterMotorConfigs.kI; 
        shooterSlot0Configs.kD = ShooterMotorConfigs.kD; 
        topLeftShooter.getConfigurator().apply(shooterSlot0Configs);
        topRightShooter.getConfigurator().apply(shooterSlot0Configs);
        bottomLeftShooter.getConfigurator().apply(shooterSlot0Configs);
        bottomRightShooter.getConfigurator().apply(shooterSlot0Configs);

        follower = new StrictFollower(CAN.topLeftShooter);
        bottomLeftShooter.setControl(follower.withLeaderID(CAN.topLeftShooter));
        follower = new StrictFollower(CAN.topRightShooter);
        bottomRightShooter.setControl(follower.withLeaderID(CAN.topRightShooter));

        topLeftShooter.getConfigurator().apply(allShooterConfig);
        topRightShooter.getConfigurator().apply(allShooterConfig);
        bottomLeftShooter.getConfigurator().apply(allShooterConfig);
        bottomRightShooter.getConfigurator().apply(allShooterConfig);

        hoodMotor = new TalonFX(Constants.CAN.hoodMotor);

        hoodConfig = new TalonFXConfiguration();

        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterMotorConfigs.StatorLimitEnable;
        hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterMotorConfigs.StatorLimit;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterMotorConfigs.SupplyLimitEnable;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterMotorConfigs.SupplyLimit;

        hoodSlot0Configs = new Slot0Configs();

        hoodSlot0Configs.kP = HoodMotorConfigs.kP; 
        hoodSlot0Configs.kI = HoodMotorConfigs.kI; 
        hoodSlot0Configs.kD = HoodMotorConfigs.kD; 

        hoodMotor.getConfigurator().apply(hoodSlot0Configs);
        hoodMotor.getConfigurator().apply(hoodConfig);
    }

    public ShootingData getShootingData(double poseX, double poseY, double velX, double velY){
        data = ShooterMath.generateRotation2d(poseX, poseY, velX, velY);
        return data;
    }

    public ShootingData getShootingData(){
        return data;
    }

    public void setLeftShooterVelocity(AngularVelocity velocity){
        VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        topLeftShooter.setControl(m_request.withVelocity(velocity.in(RotationsPerSecond)));
    }

    public void setRightShooterVelocity(AngularVelocity velocity){
         VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
         topRightShooter.setControl(m_request.withVelocity(velocity.in(RotationsPerSecond)));
    }

    public void setMasterVelocity(AngularVelocity velocity){
        setLeftShooterVelocity(velocity);
        setRightShooterVelocity(velocity);
    }
    
    public void setMasterVelocity(AngularVelocity leftVelocity, AngularVelocity rightVelocity){
        setLeftShooterVelocity(leftVelocity);
        setRightShooterVelocity(rightVelocity);
    }
    public void setHoodAngle(Angle angle) {
        PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        hoodMotor.setControl(m_request.withPosition(angle));
    }





    public class ShooterCommands{
        public ShooterCommands(){
            
        }

        
        public Command autoShoot(AngularVelocity calculatedVelocity) {
            return Commands.runOnce(()->{
                setMasterVelocity(calculatedVelocity);
            });
        }

        public Command autoShoot(AngularVelocity calculatedLeftVelocity, AngularVelocity calculatedRightVelocity) {
            return Commands.runOnce(()->{
                setMasterVelocity(calculatedLeftVelocity, calculatedRightVelocity);
            });
        }

        public Command autoAngle(Angle calculatedAngle) {
            return Commands.runOnce(()->setHoodAngle(calculatedAngle));
        }
    }
       
}

