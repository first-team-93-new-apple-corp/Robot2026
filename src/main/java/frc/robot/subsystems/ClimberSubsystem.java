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
    }

}
