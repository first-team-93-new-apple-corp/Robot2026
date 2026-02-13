package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX climberMotor;
    private TalonFXConfiguration climberMotorConfig;
    private MotionMagicVoltage m_motmag = new MotionMagicVoltage(Rotations.of(0));
    private NeutralOut neutral = new NeutralOut();


    public ClimberSubsystem() {
        climberMotor = new TalonFX(30);

        // PID Slot 0 Configuration
        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = ClimberConstants.kP; 
        slot0Configs.kI = ClimberConstants.kI;
        slot0Configs.kD = ClimberConstants.kD;

        climberMotorConfig.withSlot0(slot0Configs);

        // All of the current limits
        climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        climberMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 5;
        climberMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        // Motion Magic Configs
        climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 20;
        climberMotorConfig.MotionMagic.MotionMagicAcceleration = 40;
        climberMotorConfig.MotionMagic.MotionMagicJerk = 40;
        // climberMotorConfig.MotionMagic.MotionMagicExpo_kA = 0;
        // climberMotorConfig.MotionMagic.MotionMagicExpo_kV = 0;


        // Applies the config
        climberMotor.getConfigurator().apply(climberMotorConfig);
    }

    public ClimberCommands commands = new ClimberCommands();

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void runDistance(double distance) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        climberMotor.setControl(m_request.withPosition(distance));
    }
    public void stop() {
        climberMotor.setControl(neutral);
    }

    public class ClimberCommands {
        
        public Command Stop() {
            return runOnce(() -> stop());
        }

        public Command manualRetract() {
            return runOnce(() -> setSpeed(Constants.ClimberConstants.climberSpeed));
        }

        public Command manualExtend() {
            return runOnce(() -> setSpeed(-Constants.ClimberConstants.climberSpeed));
        }

        public Command autoExtend() {
            return runOnce(() -> runDistance(Constants.ClimberConstants.barHeight));
        }
        
        public Command autoRetract() {
            return runOnce(() -> runDistance(Constants.ClimberConstants.baseHeight));
        }
    }

}
