package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;

public class ManipulationSubsystem {
    private TalonFX bottomRollerMotor = new TalonFX(constants.ManipulationConstants.bottomRollerMotorID); // Bottom Rollers
    private TalonFX sideRollerMotor = new TalonFX(constants.ManipulationConstants.sideRollerMotorID); // Side Rollers
    private TalonFX kickerMotor = new TalonFX(constants.ManipulationConstants.kickerMotorID); // Top Rollers

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    public manipulationCommands commands = new manipulationCommands();
    /*
     * Intaking State
     * Outtaking State
     * Idle State
     * 
     */

    public ManipulationSubsystem() {
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kickerMotor.getConfigurator().apply(motorConfig);
        bottomRollerMotor.getConfigurator().apply(motorConfig);
        sideRollerMotor.getConfigurator().apply(motorConfig);
    }

    public void intake() {
        kickerMotor.set(constants.ManipulationConstants.intake.kickerSpeed);
        bottomRollerMotor.set(constants.ManipulationConstants.intake.bottomRollerSpeed);
        sideRollerMotor.set(constants.ManipulationConstants.intake.sideRollerSpeed);
    }

     public void outtake() {
        kickerMotor.set(constants.ManipulationConstants.outtake.kickerSpeed);
        bottomRollerMotor.set(constants.ManipulationConstants.outtake.bottomRollerSpeed);
        sideRollerMotor.set(constants.ManipulationConstants.outtake.sideRollerSpeed);
    }

     public void idle() {
        kickerMotor.set(constants.ManipulationConstants.idle.kickerSpeed);
        bottomRollerMotor.set(constants.ManipulationConstants.idle.bottomRollerSpeed);
        sideRollerMotor.set(constants.ManipulationConstants.idle.sideRollerSpeed);
    }
    public void shoot(){
        kickerMotor.set(constants.ManipulationConstants.shoot.kickerSpeed);
        bottomRollerMotor.set(constants.ManipulationConstants.shoot.bottomRollerSpeed);
        sideRollerMotor.set(constants.ManipulationConstants.shoot.sideRollerSpeed);
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
