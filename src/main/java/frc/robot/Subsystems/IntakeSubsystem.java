package frc.robot.Subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;

import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeRollerMotor;
    private TalonFX intakePivotMotor;

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

        // Intake Pivot Config
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
        intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
        intakePivotConfig.MotionMagic.MotionMagicAcceleration = 5;
        intakePivotConfig.MotionMagic.MotionMagicJerk = 0;

        intakePivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakePivotConfig.CurrentLimits.StatorCurrentLimit = 35;

        intakePivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakePivotConfig.Feedback.RotorToSensorRatio = 1;
        intakePivotConfig.Feedback.SensorToMechanismRatio = 45;
        intakePivotConfig.Feedback.FeedbackRotorOffset = 0;

        intakePivotMotor.getConfigurator().apply(intakePivotConfig);

        m_request = new MotionMagicVoltage(0.0).withSlot(0);

        // ** Intake Roller Config
        intakeRollerConfig = new TalonFXConfiguration();
        intakeRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeRollerConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakePivotConfig.CurrentLimits.SupplyCurrentLimit = 50;

        intakeRollerMotor.getConfigurator().apply(intakeRollerConfig);

        SmartDashboard.putData("Brake IntakePivot", commands.brakePivotMotor(true));
        SmartDashboard.putData("Coast IntakePivot", commands.brakePivotMotor(false));
        SmartDashboard.putData("IntakeZero", commands.setIntakeZero());

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

    public Angle getPivotPose() {
        return intakePivotMotor.getPosition().getValue();
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

    public void smartDash() {
        SmartDashboard.putNumber("IntakePivotPosition", intakePivotMotor.getPosition().getValue().in(Degrees));
    }

    @Override
    public void periodic() {
        smartDash();
    }

    public void log() {
        Logger.log(intakePivotMotor);
        Logger.log(intakeRollerMotor);
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

        public Command wigglePivot(Trigger trigger) {
            double stepTimeout = 0.5;
            Command sequence = Commands.sequence(
                    autoPivotDown()
                            .alongWith(Commands.waitSeconds(stepTimeout)),
                    autoPivotUp().alongWith(Commands.waitSeconds(stepTimeout)));

            return sequence.repeatedly().until(() -> !trigger.getAsBoolean());
        }

        public Command wigglePivot(Time time) {
           double stepTimeout = 0.5;
            Command sequence = Commands.sequence(
                    autoPivotDown().alongWith(Commands.waitSeconds(stepTimeout)),
                    autoPivotUp().alongWith(Commands.waitSeconds(stepTimeout))
                            .withTimeout(stepTimeout));
            return sequence.repeatedly().withTimeout(time);
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

        public Command setIntakeZero() {
            return runOnce(() -> intakePivotMotor.setPosition(0)).ignoringDisable(true);
        }

        public Command logging() {
            return Commands.run(() -> log());
        }

        public Command smartDashboard() {
            return Commands.run(() -> smartDash());
        }
    }

}
