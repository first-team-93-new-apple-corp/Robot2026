package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX climberMotor;
    private TalonFXConfiguration climberMotorConfig;
    private MotionMagicVoltage m_motmag = new MotionMagicVoltage(Rotations.of(0));
    private NeutralOut neutral = new NeutralOut();


    public ClimberSubsystem() {
        climberMotor = new TalonFX(10);
        
        // robot init, set slot 0 gains
        climberMotorConfig.Slot0.kP = 1.0;
        climberMotorConfig.Slot0.kI = 1.0;
        climberMotorConfig.Slot0.kD = 1.0;

        climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberMotor.getConfigurator().apply(climberMotorConfig);
    }

    public ClimberCommands commands = new ClimberCommands();

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void runDistance(double distance) {
        climberMotor.setPosition(distance);
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

        //Version 2 - Logan & Andrew
        
        public Command autoExtendRetract() {
            return Commands.runOnce(() -> climberMotor.setControl(m_motmag.withPosition(Rotations.of(20))));
        }
    }

}
