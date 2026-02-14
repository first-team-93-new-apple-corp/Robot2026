package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // Simulation
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Classes
    public SysID sysID = new SysID();
    public DrivetrainCommands commands = new DrivetrainCommands();

    // Alliance stuff
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    // SysID
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

   

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public class SysID implements Subsystem {
        private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // Use
                                                                                                              // default
                                                                                                              // ramp
                                                                                                              // rate (1
                                                                                                              // V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
                new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null,
                        this));

        private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default
                                                                                                        // ramp rate (1
                                                                                                        // V/s)
                Volts.of(7), // Use dynamic voltage of 7 V
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
                new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

        private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
                /* This is in radians per second², but SysId only supports "volts per second" */
                Volts.of(Math.PI / 6).per(Second),
                /* This is in radians per second, but SysId only supports "volts" */
                Volts.of(Math.PI), null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
                new SysIdRoutine.Mechanism(output -> {
                    /* output is actually radians per second, but SysId only supports "volts" */
                    setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                    /* also log the requested output for SysId */
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                }, null, this));

        /* The SysId routine to test */
        private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

        public void setSysIdRoutine(String routine) {
            switch (routine) {
                default:
                case "m_sysIdRoutineTranslation":
                    m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
                    break;
                case "m_sysIdRoutineRotation":
                    m_sysIdRoutineToApply = m_sysIdRoutineRotation;
                    break;
                case "m_sysIdRoutineSteer":
                    m_sysIdRoutineToApply = m_sysIdRoutineSteer;
                    break;
            }
        }
    }

    public class DrivetrainCommands {
        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return sysID.m_sysIdRoutineToApply.quasistatic(direction);
        }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return sysID.m_sysIdRoutineToApply.dynamic(direction);
        }

        public Command applyRequest(Supplier<SwerveRequest> request) {
            return run(() -> setControl(request.get()));
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
