package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class IntakeSubsystem extends SubsystemBase{

    private TalonFX intakeMotor;
    public IntakeCommands Commands = new IntakeCommands();
    public IntakeSubsystem() {
        intakeMotor = new TalonFX(9);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
        
    }

    public class IntakeCommands {
        public Command stop() {
            return runOnce(() -> setSpeed(0));
        }

        public Command intake() {
            return runOnce(() -> setSpeed(-0.2));
        }

        public Command outtake() {
            return runOnce(() -> setSpeed(0.2));
        }
        
    }
    
}
