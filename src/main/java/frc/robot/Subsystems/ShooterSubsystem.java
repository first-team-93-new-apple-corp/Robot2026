package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants.HoodMotorConfigs;
import frc.robot.Constants.ShooterConstants.ShooterMotorConfigs;
import frc.robot.Constants.ShooterConstants.preset;
import frc.robot.util.Logger;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingData;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

    private Slot0Configs shooterSlot0Configs;

    private AngularVelocity lastShooterSetpoint;

    private final MotionMagicVelocityVoltage m_velRequest;

    private double onTheFlyRPM = 0;

    // Telemetry registry
    // private UniversalNTLogger uniLogger;

    public ShooterSubsystem() {
        commands = new ShooterCommands();
        topLeftShooter = new TalonFX(CAN.topLeftShooter);
        topRightShooter = new TalonFX(CAN.topRightShooter);

        allShooterConfig = new TalonFXConfiguration();

        m_velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

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

        lastShooterSetpoint = RotationsPerSecond.of(0);

        // SmartDashboard.putNumber("setVelocity (RPS)", 0);
        // SmartDashboard.putNumber("setHood (Degrees)", 0);

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

    public void smartDash() {
        // DriverStation.reportWarning("Shooter S" +
        // Microseconds.of(RobotController.getTime()).in(Milliseconds), false);

        SmartDashboard.putNumber("Hood Position",
                hoodMotor.getPosition().getValue().plus(HoodMotorConfigs.offsetAngle).in(Degrees));
        // SmartDashboard.putNumber("Raw Hood",
        // hoodMotor.getPosition().getValue().in(Degrees));

        // SmartDashboard.putNumber("HoodSetpoint", lastSetpoint.in(Degrees));

        // SmartDashboard.putNumber("Velocity (avg)", getAvgVelocity());
        // SmartDashboard.putNumber("Velocity Feet/s (avg)", getAvgVelocityFeet());

        SmartDashboard.putBoolean("Shooter Ready?", shooterAtSetpoint(lastShooterSetpoint));

        // DriverStation.reportWarning("Shooter E" +
        // Microseconds.of(RobotController.getTime()).in(Milliseconds), false);
    }

    public void log() {
        Logger.log(hoodMotor);
        Logger.log(topLeftShooter);
        Logger.log(topRightShooter);
    }
    // @Override
    // public void periodic() {
    // onTheFlyRPM = SmartDashboard.getNumber("setVelocity (RPS)", 0);
    // onTheFlyHoodDegrees = SmartDashboard.getNumber("setHood (Degrees)", 0);
    // }

    public ShootingData getShootingData(double poseX, double poseY, double velX, double velY) {
        data = ShooterMath.generateRotation2d(poseX, poseY, velX, velY);
        return data;
    }
    public ShootingData getShootingDataFallback(double poseX, double poseY) {
        data = ShooterMath.fallBack(poseX, poseY);
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

    public void setShooterControl(ControlRequest signal) {
        topLeftShooter.setControl(signal);
        topRightShooter.setControl(signal);
    }

    public boolean shooterAtSetpoint(AngularVelocity velocity) {
        return topLeftShooter.getVelocity(false).isNear(velocity, RotationsPerSecond.of(1))
                && topRightShooter.getVelocity(false).isNear(velocity, RotationsPerSecond.of(1));
    }

    

    public class ShooterCommands {
        public Command autoShoot(AngularVelocity calculatedVelocity) {
            return Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(calculatedVelocity), ShooterSubsystem.this)
                            .withTimeout(0.15));
        }

        public Command autoShoot(AngularVelocity calculatedLeftVelocity, AngularVelocity calculatedRightVelocity) {
            return Commands
                    .sequence(Commands.runOnce(() -> setMasterVelocity(calculatedLeftVelocity, calculatedRightVelocity))
                            .withTimeout(0.15));
        }

        public Command autoShoot(Supplier<AngularVelocity> calculatedVelocity) {
            return Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(calculatedVelocity.get()), ShooterSubsystem.this)
                            .withTimeout(0.15));
        }

        public Command autoShoot(Supplier<AngularVelocity> calculatedLeftVelocity,
                Supplier<AngularVelocity> calculatedRightVelocity) {
            return Commands.sequence(
                    Commands.runOnce(
                            () -> setMasterVelocity(calculatedLeftVelocity.get(), calculatedRightVelocity.get()))
                            .withTimeout(0.15));
        }

        public Command stopShooter() {
            return Commands.runOnce(() -> setShooterControl(new VoltageOut(5.0)));
        }


        public Command testingShooter() {
            return Commands.runOnce(() -> setMasterVelocity(RotationsPerSecond.of(onTheFlyRPM)));
        }

        public Command velocityAndHoodNoOffset(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
            var shoot = Commands.runOnce(() -> setMasterVelocity(velocity.get()));
            return shoot.withTimeout(Milliseconds.of(100));
        }

        public Command velocityAndHood(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
            var shoot = Commands.runOnce(() -> setMasterVelocity(velocity.get()));
            return shoot.withTimeout(Milliseconds.of(100));
        }

        public Command velocityAndHoodNoOffset(Supplier<preset> point) {
            var shoot = Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(point.get().velocity())));
            return shoot.withTimeout(Milliseconds.of(100));
        }

        public Command velocityAndHood(Supplier<preset> point) {
            var shoot = Commands.sequence(
                    Commands.runOnce(() -> setMasterVelocity(point.get().velocity())));
            return shoot.withTimeout(Milliseconds.of(100));
        }

        public Command logging() {
            return Commands.run(() -> log());
        }

        public Command smartDashboard() {
            return Commands.run(() -> smartDash());
        }
    }

}
