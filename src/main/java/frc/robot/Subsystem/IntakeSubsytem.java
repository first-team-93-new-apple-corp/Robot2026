package frc.robot.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytem extends SubsystemBase {
    
    private TalonFX topIntakeMotor;
    private TalonFX bottomIntakeMotor;
    public IntakeCommands Commands = new IntakeCommands();
    private double intakeSpeed = Constants.IntakeConstants.intakeSpeed;
    private double outtakeSpeed = Constants.IntakeConstants.outtakeSpeed;
    public IntakeSubsytem() {
        topIntakeMotor = new TalonFX(10);
        bottomIntakeMotor = new TalonFX(11);
    }

    public void setSpeed(double Speed) {
        topIntakeMotor.set(Speed);
        bottomIntakeMotor.set(Speed);
    }

    public class IntakeCommands() {
        public Command stop() {
            return runOnce(() -> setSpeed(0));
        }
    }
}
