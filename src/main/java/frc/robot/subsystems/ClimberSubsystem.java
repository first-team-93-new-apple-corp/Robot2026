package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberSubsystem extends SubsystemBase {

    private TalonFX climberMotor;
    
    public ClimberSubsystem(){
        climberMotor = new TalonFX(10);
    }

    public ClimberCommands Commands = new ClimberCommands();
    
    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }
    
    public class ClimberCommands() {

        public Command Stop() {
            return runOnce(() -> setSpeed(0))
        }
        
        public Command Retract() {
            return runOnce(() -> setSpeed(Constants.ClimberConstants.climberSpeed));
        }
        
        public Command Extend () {
            return runOnce(() -> setSpeed(-Constants.ClimberConstants.climberSpeed));
        }
    }
    


    
}
