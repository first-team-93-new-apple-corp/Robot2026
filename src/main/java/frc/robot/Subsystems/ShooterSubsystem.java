package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants.HoodMotorConfigs;
import frc.robot.Constants.ShooterConstants.ShooterMotorConfigs;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingData;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterCommands commands;

    public ShootingData data;

    private TalonFX topLeftShooter;
    private TalonFX topRightShooter;
    private TalonFX bottomLeftShooter;
    private TalonFX bottomRightShooter;

    private TalonFX hoodMotor;

    private TalonFXConfiguration allShooterConfig;
    private TalonFXConfiguration hoodConfig;

    private Follower leftFollower;
    private Follower rightFollower;

    private Slot0Configs shooterSlot0Configs;

    private Slot0Configs hoodSlot0Configs;

    private DigitalInput hoodLimitSwitch;

    private Angle lastSetpoint;

    private final MotionMagicVelocityVoltage m_velRequest;

    private final MotionMagicVoltage m_volRequest;

    // private PIDController leftShooterPID = new
    // PIDController(Constants.ShooterConstants.ShooterPID_P,
    // Constants.ShooterConstants.ShooterPID_I,
    // Constants.ShooterConstants.ShooterPID_D);

    public ShooterSubsystem() {
        commands = new ShooterCommands();
        topLeftShooter = new TalonFX(Constants.CAN.topLeftShooter);
        topRightShooter = new TalonFX(Constants.CAN.topRightShooter);
        bottomLeftShooter = new TalonFX(Constants.CAN.bottomLeftShooter);
        bottomRightShooter = new TalonFX(Constants.CAN.bottomRightShooter);

        allShooterConfig = new TalonFXConfiguration();

        m_velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
        m_volRequest = new MotionMagicVoltage(0).withSlot(0);

        allShooterConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterMotorConfigs.StatorLimitEnable;
        allShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterMotorConfigs.StatorLimit;
        allShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterMotorConfigs.SupplyLimitEnable;
        allShooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterMotorConfigs.SupplyLimit;
        allShooterConfig.Feedback.RotorToSensorRatio = 1;
        allShooterConfig.Feedback.SensorToMechanismRatio = 1.25;
        allShooterConfig.Feedback.VelocityFilterTimeConstant = 0.25;
        allShooterConfig.MotionMagic.MotionMagicAcceleration = 100;
        allShooterConfig.MotionMagic.MotionMagicJerk = 100;

        shooterSlot0Configs = new Slot0Configs();

        shooterSlot0Configs.kS = ShooterMotorConfigs.kS;
        shooterSlot0Configs.kV = ShooterMotorConfigs.kV;
        shooterSlot0Configs.kP = ShooterMotorConfigs.kP;
        shooterSlot0Configs.kI = ShooterMotorConfigs.kI;
        shooterSlot0Configs.kD = ShooterMotorConfigs.kD;

        allShooterConfig.Slot0 = shooterSlot0Configs;

        leftFollower = new Follower(CAN.topLeftShooter, MotorAlignmentValue.Opposed);
        bottomLeftShooter.setControl(leftFollower.withLeaderID(CAN.topLeftShooter));
        rightFollower = new Follower(CAN.topRightShooter, MotorAlignmentValue.Opposed);
        bottomRightShooter.setControl(rightFollower.withLeaderID(CAN.topRightShooter));

        topLeftShooter.getConfigurator().apply(allShooterConfig);
        topRightShooter.getConfigurator().apply(allShooterConfig);
        bottomLeftShooter.getConfigurator().apply(allShooterConfig);
        bottomRightShooter.getConfigurator().apply(allShooterConfig);

        hoodMotor = new TalonFX(Constants.CAN.hoodMotor);

        hoodConfig = new TalonFXConfiguration();

        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterMotorConfigs.StatorLimitEnable;
        hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterMotorConfigs.StatorLimit;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterMotorConfigs.SupplyLimitEnable;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterMotorConfigs.SupplyLimit;

        hoodSlot0Configs = new Slot0Configs();

        hoodSlot0Configs.kP = HoodMotorConfigs.kP;
        hoodSlot0Configs.kI = HoodMotorConfigs.kI;
        hoodSlot0Configs.kD = HoodMotorConfigs.kD;
        hoodSlot0Configs.kS = HoodMotorConfigs.kS;

        hoodConfig.Slot0 = hoodSlot0Configs;

        hoodConfig.Feedback.FeedbackRemoteSensorID = CAN.hoodEncoder;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hoodConfig.Feedback.RotorToSensorRatio = 2;
        hoodConfig.Feedback.SensorToMechanismRatio = (360 / 20) * 0.75;

        hoodConfig.MotionMagic.MotionMagicAcceleration = 80;
        hoodConfig.MotionMagic.MotionMagicJerk = 160;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
        hoodConfig.MotionMagic.MotionMagicExpo_kV = 0.01;
        hoodConfig.MotionMagic.MotionMagicExpo_kA = 0.01;

        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodMotor.getConfigurator().apply(hoodConfig);

        hoodLimitSwitch = new DigitalInput(Constants.CAN.hoodLimitSwitch);

        lastSetpoint = Rotations.of(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hoodLimit", getHoodLimit());
        // if (getHoodLimit()) { // Shouldn't need this at all since the encoder is absolute, leave commented out
        // hoodMotor.setPosition(HoodMotorConfigs.minAngle);
        // }
        SmartDashboard.putNumber("Hood Position",
                hoodMotor.getPosition().getValue().plus(HoodMotorConfigs.offsetAngle).in(Degrees));
        SmartDashboard.putNumber("HoodSetpoint", lastSetpoint.in(Degrees));

    }

    public boolean getHoodLimit() {
        return hoodLimitSwitch.get();
    }

    public void resetHood() {
        hoodMotor.setPosition(HoodMotorConfigs.minAngle);
    }

    public ShootingData getShootingData(double poseX, double poseY, double velX, double velY) {
        data = ShooterMath.generateRotation2d(poseX, poseY, velX, velY);
        return data;
    }

    public ShootingData getShootingData() {
        return data;
    }

    public void setLeftShooterVelocity(AngularVelocity velocity) {
        topLeftShooter.setControl(m_velRequest.withVelocity(velocity.in(RotationsPerSecond)));
    }

    public void setRightShooterVelocity(AngularVelocity velocity) {
        topRightShooter.setControl(m_velRequest.withVelocity(velocity.in(RotationsPerSecond)));
    }

    public void setMasterVelocity(AngularVelocity velocity) {
        setLeftShooterVelocity(velocity);
        setRightShooterVelocity(velocity);
    }

    public void setMasterVelocity(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
        setLeftShooterVelocity(leftVelocity);
        setRightShooterVelocity(rightVelocity);
    }

    public boolean atSetpoint() {
        return hoodMotor.getPosition().getValue().isNear(lastSetpoint, Rotations.of(2));
    }

    public Angle getHoodPositionNoOffset() {
        return hoodMotor.getPosition().getValue();
    }
    public Angle getHoodPosition() {
        return hoodMotor.getPosition().getValue().plus(HoodMotorConfigs.offsetAngle);
    }

    public void setHoodPosition(Angle position) {
        if (position.lt(HoodMotorConfigs.minAngleNoOffset)) {
            position = HoodMotorConfigs.minAngleNoOffset;
        } else if (position.gt(HoodMotorConfigs.maxAngleNoOffset)) {
            position = HoodMotorConfigs.maxAngleNoOffset;
        }
        lastSetpoint = position;
        hoodMotor.setControl(m_volRequest.withPosition(position).withSlot(0));
    }

    public void setHoodPositionWithOffset(Angle position) {
        position = position.plus(HoodMotorConfigs.offsetAngle);
        if (position.lt(HoodMotorConfigs.minAngle)) {
            position = HoodMotorConfigs.minAngle;
        } else if (position.gt(HoodMotorConfigs.maxAngle)) {
            position = HoodMotorConfigs.maxAngle;
        }
        lastSetpoint = position;
        hoodMotor.setControl(m_volRequest.withPosition(position).withSlot(0));
    }

    public boolean pivotAtSetpoint(Angle setpoint) {
        return hoodMotor.getPosition().getValue().isNear(lastSetpoint, Degrees.of(1));
    }

    public class ShooterCommands {
        public Command autoShoot(AngularVelocity calculatedVelocity) {
            return Commands.runOnce(() -> {
                setMasterVelocity(calculatedVelocity);
            });
        }

        public Command autoShoot(AngularVelocity calculatedLeftVelocity, AngularVelocity calculatedRightVelocity) {
            return Commands.runOnce(() -> {
                setMasterVelocity(calculatedLeftVelocity, calculatedRightVelocity);
            });
        }

        public Command autoAngleNoOffset(Angle calculatedAngle) {
            return Commands.sequence(
                    Commands.runOnce(() -> setHoodPosition(calculatedAngle), ShooterSubsystem.this),
                    Commands.waitUntil(() -> pivotAtSetpoint(calculatedAngle)));
        }
        public Command autoAngle(Angle calculatedAngle) {
            return Commands.sequence(
                    Commands.runOnce(() -> setHoodPositionWithOffset(calculatedAngle), ShooterSubsystem.this),
                    Commands.waitUntil(() -> pivotAtSetpoint(calculatedAngle.plus(HoodMotorConfigs.offsetAngle))));
        }

        public Command stopShooter() {
            return Commands.runOnce(() -> setMasterVelocity(RotationsPerSecond.of(0)));
        }

        public Command stopHood() {
            return Commands.runOnce(() -> hoodMotor.setControl(m_volRequest.withPosition(getHoodPositionNoOffset())));
        }
    }

}
