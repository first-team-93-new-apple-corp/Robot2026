package frc.robot.Subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeRollerMotor;
    private TalonFX intakePivotMotor;

    private CANcoder pivotEncoder;

    private TalonFXConfiguration intakePivotConfig;
    private TalonFXConfiguration intakeRollerConfig;

    private Slot0Configs slot0;

    private final MotionMagicVoltage m_request;

    private Angle lastSetpoint;

    public IntakeCommands commands;

    public IntakeSubsystem() {
        commands = new IntakeCommands();

        lastSetpoint = Rotations.of(0);

        intakeRollerMotor = new TalonFX(CAN.intakeRoller);
        intakePivotMotor = new TalonFX(CAN.intakePivot);


        // ** Intake Pivot Config
        intakePivotConfig = new TalonFXConfiguration();

        // PID Slot 0 Configuration
        slot0 = new Slot0Configs();
        slot0.kP = IntakeConstants.pivotkP;
        slot0.kI = IntakeConstants.pivotkI;
        slot0.kD = IntakeConstants.pivotkD;
        slot0.kV = IntakeConstants.pivotkV;
        slot0.kA = IntakeConstants.pivotkA;
        slot0.kG = IntakeConstants.pivotkG;
        slot0.kS = IntakeConstants.pivotkS;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        intakePivotConfig.withSlot0(slot0);
        intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakePivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Motion Magic Configs
        intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
        intakePivotConfig.MotionMagic.MotionMagicAcceleration = 0.25;
        intakePivotConfig.MotionMagic.MotionMagicJerk = 0.5;

        intakePivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakePivotConfig.CurrentLimits.StatorCurrentLimit = 40;

        intakePivotConfig.Feedback.FeedbackRemoteSensorID = CAN.intakePivotEncoder;
        intakePivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        intakePivotConfig.Feedback.RotorToSensorRatio = 27;
        intakePivotConfig.Feedback.FeedbackRotorOffset = 0;

        intakePivotMotor.getConfigurator().apply(intakePivotConfig);

        m_request = new MotionMagicVoltage(0.0).withSlot(0);

        // ** Intake Roller Config
        intakeRollerConfig = new TalonFXConfiguration();
        intakeRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeRollerMotor.getConfigurator().apply(intakeRollerConfig);

        SmartDashboard.putData("Brake IntakePivot", commands.brakePivotMotor(true));
        SmartDashboard.putData("Coast IntakePivot", commands.brakePivotMotor(false));
    }

    public void setRollerSpeed(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setPivotSpeed(double speed) {
        intakePivotMotor.set(speed);
    }

    public boolean atSetpoint() {
        return intakePivotMotor.getPosition().getValue().isNear(lastSetpoint, Rotations.of(2));
    }

    public Angle getPivotPoseRaw() {
        return pivotEncoder.getAbsolutePosition().getValue();
    }

    public Angle getPivotPose() {
        return getPivotPoseRaw();
    }

    public void setPivotPosition(Angle position) {
        if (position.lt(IntakeConstants.pivotDownPosition)) {
            position = IntakeConstants.pivotDownPosition;
        } else if (position.gt(IntakeConstants.pivotUpPosition)) {
            position = IntakeConstants.pivotUpPosition;
        }
        lastSetpoint = position;
        intakePivotMotor.setControl(m_request.withPosition(lastSetpoint));
    }

    public boolean pivotAtSetpoint(Angle setpoint) {
        return intakePivotMotor.getPosition().getValue().isNear(lastSetpoint, Rotations.of(2));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakePivotPosition", intakePivotMotor.getPosition().getValue().in(Degrees));
        SmartDashboard.putNumber("IntakePivotSetpoint", lastSetpoint.in(Degrees));
        SmartDashboard.putNumber("IntakePivotCurrentStator", intakePivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("IntakePivotCurrentSupply", intakePivotMotor.getSupplyCurrent().getValueAsDouble());
    }

    public class IntakeCommands {
        public Command stop() {
            return runOnce(() -> setRollerSpeed(0));
        }

        public Command intake() {
            return runOnce(() -> setRollerSpeed(IntakeConstants.intakeSpeed));
        }

        public Command outtake() {
            return runOnce(() -> setRollerSpeed(IntakeConstants.outtakeSpeed));
        }

        public Command idle() {
            return runOnce(() -> setRollerSpeed(IntakeConstants.idleSpeed));
        }

        public Command manPivotUp() {
            return runOnce(() -> setPivotSpeed(IntakeConstants.pivotUpSpeed));
        }

        public Command manPivotDown() {
            return runOnce(() -> setPivotSpeed(IntakeConstants.pivotDownSpeed));
        }

        public Command manPivotStop() {
            return runOnce(() -> setPivotSpeed(0));
        }

        public Command autoPivotUp() {
            return runOnce(() -> setPivotPosition(IntakeConstants.pivotUpPosition));
        }

        public Command autoPivotDown() {
            return runOnce(() -> setPivotPosition(IntakeConstants.pivotDownPosition));
        }

        public Command autoPivotMiddle() {
            return runOnce(() -> setPivotPosition(IntakeConstants.pivotMiddlePosition));
        }

        public Command wigglePivot() {
            double delay = 0.5; // seconds
            Command moveToMiddle = Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotMiddlePosition));
            Command moveToDown = Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotDownPosition));
            Command moveToUp = Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotUpPosition));
            Command wait = Commands.waitSeconds(delay);
            Command wait2 = Commands.waitSeconds(delay);
            Command sequence = Commands.sequence(moveToMiddle, wait, moveToDown, wait2, moveToUp);
            // Command parallel = Commands.parallel(sequence, intake());
            return sequence;
        }

        public Command brakePivotMotor(boolean brake) {
            return runOnce(() -> {
                if (brake) {
                    intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                    intakePivotMotor.getConfigurator().apply(intakePivotConfig);
                } else {
                    intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    intakePivotMotor.getConfigurator().apply(intakePivotConfig);
                }

            }).ignoringDisable(true);
        }
    }

}
