package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CAN;
import frc.robot.Constants.ShooterConstants.ShooterMotorConfigs;
import frc.robot.util.Logger;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingData;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public ShooterCommands commands;

    public ShootingData data;

    /***
     * Left Motor
     */
    private TalonFX masterMotor;

    /***
     * Right Motor
     */
    private TalonFX followerMotor;

    private TalonFXConfiguration masterConf;
    private TalonFXConfiguration follow;

    private Slot0Configs slot0;

    private AngularVelocity lastShooterSetpoint;

    private final MotionMagicVelocityVoltage m_velRequest;

    private double onTheFlyRPM = 0;

    // Telemetry registry
    // private UniversalNTLogger uniLogger;

    public ShooterSubsystem() {
        commands = new ShooterCommands();
        masterMotor = new TalonFX(CAN.leftShooter);
        followerMotor = new TalonFX(CAN.rightShooter);

        masterConf = new TalonFXConfiguration();
        follow = new TalonFXConfiguration();

        m_velRequest = new MotionMagicVelocityVoltage(0).withSlot(0);

        masterConf.CurrentLimits.StatorCurrentLimitEnable = ShooterMotorConfigs.StatorLimitEnable;
        masterConf.CurrentLimits.StatorCurrentLimit = ShooterMotorConfigs.StatorLimit;
        masterConf.CurrentLimits.SupplyCurrentLimitEnable = ShooterMotorConfigs.SupplyLimitEnable;
        masterConf.CurrentLimits.SupplyCurrentLimit = ShooterMotorConfigs.SupplyLimit;
        masterConf.Feedback.RotorToSensorRatio = 1;
        masterConf.Feedback.SensorToMechanismRatio = 18 / 24; // teeth
        masterConf.Feedback.VelocityFilterTimeConstant = 0.05;
        masterConf.MotionMagic.MotionMagicAcceleration = 15;
        masterConf.MotionMagic.MotionMagicJerk = 20;

        slot0 = new Slot0Configs();

        slot0.kS = ShooterMotorConfigs.kS;
        slot0.kV = ShooterMotorConfigs.kV;
        slot0.kA = ShooterMotorConfigs.kA;
        slot0.kP = ShooterMotorConfigs.kP;
        slot0.kI = ShooterMotorConfigs.kI;
        slot0.kD = ShooterMotorConfigs.kD;

        masterConf.Slot0 = slot0;
        masterConf.MotorOutput.PeakReverseDutyCycle = 0.0;
        masterConf.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        masterConf.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        masterConf.Voltage.PeakReverseVoltage = 0.0;
        masterConf.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // allShooterConfig.

        masterMotor.getConfigurator().apply(masterConf);

        follow.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        followerMotor.getConfigurator().apply(follow);

        followerMotor.setControl(new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        lastShooterSetpoint = RotationsPerSecond.of(0);

        // SmartDashboard.putNumber("setVelocity (RPS)", 0);

        // Initialize universal telemetry logger
        // uniLogger = new UniversalNTLogger("ShooterMirror");

        // // Register TalonFX motors and useful values
        // uniLogger.registerTalonFX("topLeft", topLeftShooter);
        // uniLogger.registerTalonFX("topRight", topRightShooter);
        // uniLogger.registerDouble("avg/velocity", () -> getAvgVelocity());
        // uniLogger.registerDouble("setpoint/rps", () ->
        // SmartDashboard.getNumber("setVelocity (RPS)", 0));

        // // Cap signals per flush to avoid saturation
        // uniLogger.setMaxSignalsPerFlush(10);
    }

    public double getAvgVelocity() {
        return (masterMotor.getVelocity().getValue().in(RotationsPerSecond)
                + followerMotor.getVelocity().getValue().in(RotationsPerSecond)) / 2;
    }

    public double getAvgVelocityFeet() {
        return ((getAvgVelocity()) * 4 * Math.PI) / 12;
    }

    public void smartDash() {
        // DriverStation.reportWarning("Shooter S" +
        // Microseconds.of(RobotController.getTime()).in(Milliseconds), false);

        // SmartDashboard.putNumber("Velocity (avg)", getAvgVelocity());
        // SmartDashboard.putNumber("Velocity Feet/s (avg)", getAvgVelocityFeet());

        SmartDashboard.putBoolean("Shooter Ready?", shooterAtSetpoint(lastShooterSetpoint));

        // DriverStation.reportWarning("Shooter E" +
        // Microseconds.of(RobotController.getTime()).in(Milliseconds), false);
    }

    public void log() {
        Logger.log(masterMotor);
        Logger.log(followerMotor);
    }

    // TODO Uncomment this for tuning
    // @Override
    // public void periodic() {
    // onTheFlyRPM = SmartDashboard.getNumber("setVelocity (RPS)", 0);
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

 public void setShooterVelocity(AngularVelocity velocity) {
    double targetRPS = velocity.in(RotationsPerSecond);
    // double currentRPS = masterMotor.getVelocity().getValue().in(RotationsPerSecond);
    masterMotor.setControl(m_velRequest.withVelocity(RotationsPerSecond.of(targetRPS)));
    lastShooterSetpoint = velocity;
}

    public void setShooterControl(ControlRequest signal) {
        masterMotor.setControl(signal);
    }

    public boolean shooterAtSetpoint(AngularVelocity velocity) {
        return masterMotor.getVelocity(false).isNear(velocity, RotationsPerSecond.of(1))
                && followerMotor.getVelocity(false).isNear(velocity, RotationsPerSecond.of(1));
    }

    public class ShooterCommands {
        public Command autoShoot(AngularVelocity calculatedVelocity) {
            return Commands.run(() -> setShooterVelocity(calculatedVelocity), ShooterSubsystem.this);
        }

        public Command autoShoot(Supplier<AngularVelocity> calculatedVelocity) {
            return Commands.run(() -> setShooterVelocity(calculatedVelocity.get()), ShooterSubsystem.this);
        }

        public Command stopShooter() {
            return Commands.runOnce(() -> setShooterVelocity(ShooterMotorConfigs.idleSpeed));
        }

        public Command testingShooter() {
            return Commands.runOnce(() -> setShooterVelocity(RotationsPerSecond.of(onTheFlyRPM)));
        }

        public Command logging() {
            return Commands.run(() -> log());
        }

        public Command smartDashboard() {
            return Commands.run(() -> smartDash());
        }
    }

}
