package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem.ClimberCommands;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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
        // m_Encoder = new DutyCycleEncoder(10);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ClimberConstants.kP; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = ClimberConstants.kI; // no output for integrated error
        slot0Configs.kD = ClimberConstants.kD; // A velocity of 1 rps results in 0.1 V output
        climberMotor.getConfigurator().apply(slot0Configs);
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
    }

}
