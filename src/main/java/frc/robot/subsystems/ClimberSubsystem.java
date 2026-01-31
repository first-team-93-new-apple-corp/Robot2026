package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX.DutyCycleOut;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX climberMotor;
    private TalonFXConfig climbermotorConfig;


    public ClimberSubsystem() {
        climberMotor = new TalonFX(10);
        m_Encoder = new DutyCycleEncoder(10);
    }

    public ClimberCommands Commands = new ClimberCommands();

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void runDistance(double distance) {
        climbermotor.setPosition(distance);
    }

    public class ClimberCommands {

        public Command Retract() {
            return run(()-> setPosition(Constants.ClimberConstants.barHeight));
        }

        public Command Extend(){
            return run(()-> setPosition(-Constants.ClimberConstants.barHeight));
        }
        
        public Command Stop() {
            return runOnce(() -> setSpeed(0));
        }

        public Command manualRetract() {
            return runOnce(() -> setSpeed(Constants.ClimberConstants.climberSpeed));
        }

        public Command manualExtend() {
            return runOnce(() -> setSpeed(-Constants.ClimberConstants.climberSpeed));
        }

        //Version 2 - Logan & Andrew
        
        public Command autoExtendRetract() {
        // robot init, set slot 0 gains
        m_motor.config_kF(0, 0.05, 50);
        m_motor.config_kP(0, 0.046, 50);
        m_motor.config_kI(0, 0.0002, 50);
        m_motor.config_kD(0, 4.2, 50);

        // enable voltage compensation
        m_motor.configVoltageComSaturation(12);
        m_motor.enableVoltageCompensation(true);

        // periodic, run velocity control with slot 0 configs,
        // target velocity of 50 rps (10240 ticks/100ms)
        m_motor.selectProfileSlot(0, 0);
        m_motor.setControl(m_motmag.withPosition(200));
        }
    }

}
