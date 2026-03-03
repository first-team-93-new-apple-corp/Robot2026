package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class ManipulationSubsystem {
    private TalonFX bottomRollerMotor = new TalonFX(Constants.ManipulationConstants.bottomRollerMotorID); // Bottom Rollers
    private TalonFX sideRollerMotor = new TalonFX(Constants.ManipulationConstants.sideRollerMotorID); // Side Rollers
    private TalonFX kickerMotor = new TalonFX(Constants.ManipulationConstants.kickerMotorID); // Top Rollers

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    public manipulationCommands commands = new manipulationCommands();
    /*
     * Intaking State
     * Outtaking State
     * Idle State
     * 
     */

    public ManipulationSubsystem() {
        motorConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kickerMotor.getConfigurator().apply(motorConfig);
        bottomRollerMotor.getConfigurator().apply(motorConfig);
        sideRollerMotor.getConfigurator().apply(motorConfig);
    }

    public void intake() {
        kickerMotor.set(Constants.ManipulationConstants.intake.kickerSpeed);
        bottomRollerMotor.set(Constants.ManipulationConstants.intake.bottomRollerSpeed);
        sideRollerMotor.set(Constants.ManipulationConstants.intake.sideRollerSpeed);
    }

     public void outtake() {
        kickerMotor.set(Constants.ManipulationConstants.outtake.kickerSpeed);
        bottomRollerMotor.set(Constants.ManipulationConstants.outtake.bottomRollerSpeed);
        sideRollerMotor.set(Constants.ManipulationConstants.outtake.sideRollerSpeed);
    }

     public void idle() {
        kickerMotor.set(Constants.ManipulationConstants.idle.kickerSpeed);
        bottomRollerMotor.set(Constants.ManipulationConstants.idle.bottomRollerSpeed);
        sideRollerMotor.set(Constants.ManipulationConstants.idle.sideRollerSpeed);
    }
    public void shoot(){
        kickerMotor.set(Constants.ManipulationConstants.shoot.kickerSpeed);
        bottomRollerMotor.set(Constants.ManipulationConstants.shoot.bottomRollerSpeed);
        sideRollerMotor.set(Constants.ManipulationConstants.shoot.sideRollerSpeed);
    }
    public class manipulationCommands {
        public Command intakeCommand() {
            return Commands.runOnce(() -> intake());
        }
        public Command outtakeCommand() {
            return Commands.runOnce(() -> outtake());
        }
        public Command idleCommand() {
            return Commands.runOnce(() -> idle());
        }
        public Command shootCommand(){
            return Commands.runOnce(()->shoot());
        }
    }
}
