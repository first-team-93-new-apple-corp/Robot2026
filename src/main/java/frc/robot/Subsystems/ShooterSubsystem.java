package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants.HoodMotorConfigs;
import frc.robot.Constants.ShooterConstants.ShooterMotorConfigs;
import frc.robot.Subsystems.auto.AutoConstants.PresetShootingPoint;
import frc.robot.util.Logger;
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

    private TalonFX hoodMotor;

    private TalonFXConfiguration allShooterConfig;
    private TalonFXConfiguration hoodConfig;

    private Slot0Configs shooterSlot0Configs;

    private Slot0Configs hoodSlot0Configs;

    private DigitalInput hoodLimitSwitch;

    private Angle lastSetpoint;
    private AngularVelocity lastShooterSetpoint;

    private final MotionMagicVelocityVoltage m_velRequest;

    private final MotionMagicVoltage m_volRequest;

    private double onTheFlyHoodDegrees = 0;
    private double onTheFlyRPM = 0;

    // Telemetry registry
    // private UniversalNTLogger uniLogger;

    public ShooterSubsystem() {
        commands = new ShooterCommands();
        topLeftShooter = new TalonFX(Constants.CAN.topLeftShooter);
        topRightShooter = new TalonFX(Constants.CAN.topRightShooter);

        allShooterConfig = new TalonFXConfiguration();

        m_velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
        m_volRequest = new MotionMagicVoltage(0).withSlot(0);

        allShooterConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterMotorConfigs.StatorLimitEnable;
        allShooterConfig.CurrentLimits.StatorCurrentLimit = ShooterMotorConfigs.StatorLimit;
        allShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterMotorConfigs.SupplyLimitEnable;
        allShooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterMotorConfigs.SupplyLimit;
        allShooterConfig.Feedback.RotorToSensorRatio = 1;
        allShooterConfig.Feedback.SensorToMechanismRatio = 18 / 24; // teeth
        allShooterConfig.Feedback.VelocityFilterTimeConstant = 0.005;
        allShooterConfig.MotionMagic.MotionMagicAcceleration = 50;
        allShooterConfig.MotionMagic.MotionMagicJerk = 100;

        shooterSlot0Configs = new Slot0Configs();

        shooterSlot0Configs.kS = ShooterMotorConfigs.kS;
        shooterSlot0Configs.kV = ShooterMotorConfigs.kV;
        shooterSlot0Configs.kA = ShooterMotorConfigs.kA;
        shooterSlot0Configs.kP = ShooterMotorConfigs.kP;
        shooterSlot0Configs.kI = ShooterMotorConfigs.kI;
        shooterSlot0Configs.kD = ShooterMotorConfigs.kD;

        allShooterConfig.Slot0 = shooterSlot0Configs;

        topLeftShooter.getConfigurator().apply(allShooterConfig);
        topRightShooter.getConfigurator().apply(allShooterConfig);

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
        hoodSlot0Configs.kA = HoodMotorConfigs.kA;
        hoodSlot0Configs.kV = HoodMotorConfigs.kV;

        hoodConfig.Slot0 = hoodSlot0Configs;

        hoodConfig.Feedback.FeedbackRemoteSensorID = CAN.hoodEncoder;
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hoodConfig.Feedback.RotorToSensorRatio = 2;
        hoodConfig.Feedback.SensorToMechanismRatio = (360 / 20) * 0.75;
        // hoodConfig.Feedback.FeedbackRotorOffset = -0.117;

        hoodConfig.MotionMagic.MotionMagicAcceleration = 250;
        hoodConfig.MotionMagic.MotionMagicJerk = 500;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 20;
        hoodConfig.MotionMagic.MotionMagicExpo_kV = 0.01;
        hoodConfig.MotionMagic.MotionMagicExpo_kA = 0.01;

        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodMotor.getConfigurator().apply(hoodConfig);

        lastSetpoint = Rotations.of(0);
        lastShooterSetpoint = RotationsPerSecond.of(0);

        SmartDashboard.putNumber("setVelocity (RPS)", 0);
        SmartDashboard.putNumber("setHood (Degrees)", 0);

        // Initialize universal telemetry logger
        // uniLogger = new UniversalNTLogger("ShooterMirror");

        // // Register TalonFX motors and useful values
        // uniLogger.registerTalonFX("topLeft", topLeftShooter);
        // uniLogger.registerTalonFX("topRight", topRightShooter);
        // uniLogger.registerDouble("hood/position", () ->
        // hoodMotor.getPosition().getValue().in(Degrees));
        // uniLogger.registerDouble("avg/velocity", () -> getAvgVelocity());
        // uniLogger.registerDouble("setpoint/rps", () ->
        // SmartDashboard.getNumber("setVelocity (RPS)", 0));
        // uniLogger.registerBoolean("hood/limit", () -> getHoodLimit());

        // // Cap signals per flush to avoid saturation
        // uniLogger.setMaxSignalsPerFlush(10);
        // resetHood();
    }

    public double getAvgVelocity() {
        return (topLeftShooter.getVelocity().getValue().in(RotationsPerSecond)
                + topRightShooter.getVelocity().getValue().in(RotationsPerSecond)) / 2;
    }

    public double getAvgVelocityFeet() {
        return ((getAvgVelocity()) * 4 * Math.PI) / 12;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Hood Position", hoodMotor.getPosition().getValue().plus(HoodMotorConfigs.offsetAngle).in(Degrees));
        // SmartDashboard.putNumber("Raw Hood", hoodMotor.getPosition().getValue().in(Degrees));

        // SmartDashboard.putNumber("HoodSetpoint", lastSetpoint.in(Degrees));

        // SmartDashboard.putNumber("Velocity (avg)", getAvgVelocity());
        // SmartDashboard.putNumber("Velocity Feet/s (avg)", getAvgVelocityFeet());

        // onTheFlyRPM = SmartDashboard.getNumber("setVelocity (RPS)", 0);
        // onTheFlyHoodDegrees = SmartDashboard.getNumber("setHood (Degrees)", 0);

        SmartDashboard.putBoolean("Shooter Ready?", shooterAtSetpoint(lastShooterSetpoint));

        // if (uniLogger != null) uniLogger.flushAll();

        Logger.log(hoodMotor);
        Logger.log(topLeftShooter);
        Logger.log(topRightShooter);
    }

    public boolean getHoodLimit() {
        return hoodLimitSwitch.get();
    }

    // public void resetHood() {
    // hoodMotor.setPosition(HoodMotorConfigs.minAngleNoOffset);
    // System.out.println("Rest HOod!!!!!!**********************");
    // }

    public ShootingData getShootingData(double poseX, double poseY, double velX, double velY) {
        data = ShooterMath.generateRotation2d(poseX, poseY, velX, velY);
        return data;
    }

    public ShootingData getShootingData() {
        return data;
    }

    public void setLeftShooterVelocity(AngularVelocity velocity) {
        topLeftShooter.setControl(m_velRequest
                .withVelocity(velocity.times(ShooterMotorConfigs.ShootToFlyGearRatio).in(RotationsPerSecond)));
        lastShooterSetpoint = velocity;
    }

    public void setRightShooterVelocity(AngularVelocity velocity) {
        topRightShooter.setControl(m_velRequest
                .withVelocity(velocity.times(ShooterMotorConfigs.ShootToFlyGearRatio).in(RotationsPerSecond)));
        lastShooterSetpoint = velocity;
    }

    public void setMasterVelocity(AngularVelocity velocity) {
        setLeftShooterVelocity(velocity);
        setRightShooterVelocity(velocity);
    }

    public void setMasterVelocity(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
        setLeftShooterVelocity(leftVelocity);
        setRightShooterVelocity(rightVelocity);
    }

    public Angle getHoodPositionNoOffset() {
        return hoodMotor.getPosition().getValue();
    }

    public Angle getHoodPosition() {
        return hoodMotor.getPosition().getValue().plus(HoodMotorConfigs.offsetAngle);
    }

    public void setShooterControl(ControlRequest signal) {
        topLeftShooter.setControl(signal);
        topRightShooter.setControl(signal);
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
        position = position.minus(HoodMotorConfigs.offsetAngle);
        if (position.lt(HoodMotorConfigs.minAngleNoOffset)) {
            position = HoodMotorConfigs.minAngleNoOffset;
        } else if (position.gt(HoodMotorConfigs.maxAngleNoOffset)) {
            position = HoodMotorConfigs.maxAngleNoOffset;
        }
        lastSetpoint = position;
        hoodMotor.setControl(m_volRequest.withPosition(position).withSlot(0));
    }

    public boolean hoodAtSetpoint(Angle setpoint) {
        return hoodMotor.getPosition().getValue().isNear(lastSetpoint, Degrees.of(1));
    }

    public boolean shooterAtSetpoint(AngularVelocity velocity) {
        return topLeftShooter.getVelocity(false).isNear(velocity, RotationsPerSecond.of(1))
                && topRightShooter.getVelocity(false).isNear(velocity, RotationsPerSecond.of(1));
    }

    public class ShooterCommands {
        public Command autoShoot(AngularVelocity calculatedVelocity) {
            return Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(calculatedVelocity), ShooterSubsystem.this),
                    Commands.waitUntil(() -> shooterAtSetpoint(calculatedVelocity)));
        }

        public Command autoShoot(AngularVelocity calculatedLeftVelocity, AngularVelocity calculatedRightVelocity) {
            return Commands.runOnce(() -> {
                setMasterVelocity(calculatedLeftVelocity, calculatedRightVelocity);
            });
        }

        public Command autoShoot(Supplier<AngularVelocity> calculatedVelocity) {
            return Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(calculatedVelocity.get()), ShooterSubsystem.this));
            // Commands.waitUntil(() -> shooterAtSetpoint(calculatedVelocity.get())));
        }

        public Command autoShoot(Supplier<AngularVelocity> calculatedLeftVelocity,
                Supplier<AngularVelocity> calculatedRightVelocity) {
            return Commands.runOnce(() -> {
                setMasterVelocity(calculatedLeftVelocity.get(), calculatedRightVelocity.get());
            });
        }

        public Command autoAngleNoOffset(Angle calculatedAngle) {
            return Commands.sequence(
                    Commands.runOnce(() -> setHoodPosition(calculatedAngle), ShooterSubsystem.this),
                    Commands.waitUntil(() -> hoodAtSetpoint(calculatedAngle)));
        }

        public Command autoAngle(Angle calculatedAngle) {
            return Commands.sequence(
                    Commands.runOnce(() -> setHoodPositionWithOffset(calculatedAngle), ShooterSubsystem.this),
                    Commands.waitUntil(() -> hoodAtSetpoint(calculatedAngle.plus(HoodMotorConfigs.offsetAngle))));
        }

        public Command autoAngleNoOffset(Supplier<Angle> calculatedAngle) {
            return Commands.sequence(
                    Commands.runOnce(() -> setHoodPosition(calculatedAngle.get()), ShooterSubsystem.this),
                    Commands.waitUntil(() -> hoodAtSetpoint(calculatedAngle.get())));
        }

        public Command autoAngle(Supplier<Angle> calculatedAngle) {
            return Commands.sequence(
                    Commands.runOnce(() -> setHoodPositionWithOffset(calculatedAngle.get()), ShooterSubsystem.this),
                    Commands.waitUntil(() -> hoodAtSetpoint(calculatedAngle.get().plus(HoodMotorConfigs.offsetAngle))));
        }

        public Command stopShooter() {
            return Commands.runOnce(() -> setShooterControl(new VoltageOut(5.0)));
        }

        // public Command stopHood() {
        // return Commands.runOnce(() ->
        // hoodMotor.setControl(m_volRequest.withPosition(getHoodPositionNoOffset())));
        // }

        public Command testingHood() {
            return Commands.runOnce(() -> setHoodPosition(Degrees.of(onTheFlyHoodDegrees)));
        }

        public Command testingShooter() {
            return Commands.runOnce(() -> setMasterVelocity(RotationsPerSecond.of(onTheFlyRPM)));
        }

        public Command velocityAndHoodNoOffset(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
            var anglecmd = Commands.sequence(
                    Commands.runOnce(() -> setHoodPosition(angle.get())),
                    Commands.waitUntil(() -> hoodAtSetpoint(angle.get())));
            var shoot = Commands.runOnce(() -> setMasterVelocity(velocity.get()));
            return anglecmd.alongWith(shoot);
        }

        public Command velocityAndHood(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
            var anglecmd = Commands.sequence(
                    Commands.runOnce(() -> setHoodPositionWithOffset(angle.get())),
                    Commands.waitUntil(() -> hoodAtSetpoint(angle.get().plus(HoodMotorConfigs.offsetAngle))));
            var shoot = Commands.runOnce(() -> setMasterVelocity(velocity.get()));
            return anglecmd.alongWith(shoot);
        }

        public Command velocityAndHoodNoOffset(Supplier<PresetShootingPoint> point) {
            var angle = Commands.sequence(
                    Commands.runOnce(() -> setHoodPosition(point.get().hoodAngle())),
                    Commands.waitUntil(() -> hoodAtSetpoint(point.get().hoodAngle())));
            var shoot = Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(point.get().velocity())));
            return angle.alongWith(shoot);
        }

        public Command velocityAndHood(Supplier<PresetShootingPoint> point) {
            var angle = Commands.sequence(
                    Commands.runOnce(() -> setHoodPositionWithOffset(point.get().hoodAngle())),
                    Commands.waitUntil(
                            () -> hoodAtSetpoint(point.get().hoodAngle().plus(HoodMotorConfigs.offsetAngle))));
            var shoot = Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(point.get().velocity())));
            return angle.alongWith(shoot);
        }
    }

}
