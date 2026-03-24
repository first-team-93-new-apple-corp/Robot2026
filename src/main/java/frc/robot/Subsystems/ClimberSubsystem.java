// package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.ClimberConstants;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.StaticBrake;
// import com.ctre.phoenix6.hardware.TalonFX;

// public class ClimberSubsystem extends SubsystemBase {
    
//     private TalonFX climberMotor;
//     private TalonFXConfiguration climberMotorConfig;
//     private StaticBrake neutral = new StaticBrake();
//     private DigitalInput climberLimitSwitch = new DigitalInput(9);
//     private boolean HasReset = false;
//     final MotionMagicVoltage m_request;

//     public ClimberSubsystem() {
//         climberMotor = new TalonFX(Constants.CAN.climber);

//         // PID Slot 0 Configuration
//         var slot0Configs = new Slot0Configs();

//         slot0Configs.kP = ClimberConstants.kP; 
//         slot0Configs.kI = ClimberConstants.kI;
//         slot0Configs.kD = ClimberConstants.kD;

//         climberMotorConfig = new TalonFXConfiguration();

//         climberMotorConfig.withSlot0(slot0Configs);

//         // All of the current limits
//         climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
//         climberMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
//         climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
//         climberMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 5;
//         climberMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

//         // Motion Magic Configs
//         climberMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
//         climberMotorConfig.MotionMagic.MotionMagicAcceleration = 160;
//         climberMotorConfig.MotionMagic.MotionMagicJerk = 160;
//         // climberMotorConfig.MotionMagic.MotionMagicExpo_kA = 0;
//         // climberMotorConfig.MotionMagic.MotionMagicExpo_kV = 0;


//         // Applies the config
//         climberMotor.getConfigurator().apply(climberMotorConfig);
//         m_request = new MotionMagicVoltage(0.0).withSlot(0);



        
//     }

//     @Override
//     public void periodic(){
//         resetEncoderIfAtBottom();
//     }

//     public ClimberCommands commands = new ClimberCommands();

//     public void setSpeed(double speed) {
//         climberMotor.set(speed);
//     }

//     public void runDistance(double distance) {
//         climberMotor.setControl(m_request.withPosition(distance));
//     }
//     public void stop() {
//         climberMotor.setControl(neutral);
//     }
//     public boolean isAtBottom() {
//         return !climberLimitSwitch.get();
//     }
//     public void resetEncoderIfAtBottom(){
//         if (!climberLimitSwitch.get() && !HasReset){
//             climberMotor.setPosition(3);
//             HasReset = true;
//         }
//     }
//     public class ClimberCommands {
        
//         public Command Stop() {
//             return runOnce(() -> stop());
//         }

//         public Command manualRetract() {
//             return runOnce(() -> setSpeed(Constants.ClimberConstants.climberSpeed));
//         }

//         public Command manualExtend() {
//             return runOnce(() -> setSpeed(-Constants.ClimberConstants.climberSpeed));
//         }

//         public Command autoExtend() {
//            return runOnce(() -> runDistance(Constants.ClimberConstants.barHeight));
//         // return runOnce(() -> System.out.println("Testing 1"));
//         }
        
//         public Command autoRetract() {
//             return runOnce(() -> runDistance(Constants.ClimberConstants.baseHeight));
//         }
//         public Command resetEncoder() {
//             return run(() -> climberMotor.setPosition(0));
//          }
//         }
// }
