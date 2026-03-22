package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
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

    // Pose Correction Stuffs
    private final PIDController xController = new PIDController(3.0, 0, 0);
    private final PIDController yController = new PIDController(3.0, 0, 0);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            4.0,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.Swerve.MaxSpeed, 5));

    private final HolonomicDriveController snapController = new HolonomicDriveController(xController, yController,
            thetaController);
    public final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone) // Add a
                                                                                                   // 10%
                                                                                                   // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive
                                                                     // motors
            .withHeadingPID(Constants.ShooterConstants.HeadingController.kP
            ,Constants.ShooterConstants.HeadingController.kI
            , Constants.ShooterConstants.HeadingController.kD);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Override setControl so we can log the exact SwerveRequest being applied to
     * the drivetrain. This helps confirm what the AutoBuilder consumer and other
     * callers are sending to the hardware layer (and is especially useful when
     * the robot appears not to move).
     */
    @Override
    public void setControl(SwerveRequest request) {
        // try {
        //     // Log the concrete request type and the request.toString() so we can
        //     // see what the drivetrain received in the logs. Flushing ensures the
        //     // text appears promptly in the roboRIO console.
        //     String cls = request == null ? "null" : request.getClass().getSimpleName();
        //     System.out.printf("[Drivetrain.setControl] %s %s\\n", cls, request == null ? "null" : request.toString());
        //     System.out.flush();
        // } catch (Exception e) {
        //     System.out.println("[Drivetrain.setControl] (toString failed)");
        //     System.out.flush();
        // }
        super.setControl(request);
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

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        // This helper creates a Command that repeatedly calls setControl(...) with
        // the provided SwerveRequest supplier. It's used throughout RobotContainer
        // and AutoBuilder to turn SwerveRequest objects into scheduled behavior.
        // Important notes:
        // - The supplier will be invoked on each scheduler run while the command
        //   is active. Keep the supplier lightweight.
        // - The returned command will require this drivetrain (caller should
        //   consider command requirements to avoid conflicts).
        return run(() -> this.setControl(request.get()));
    }

    public void snapToPose(Pose2d targetPose) {

        Pose2d currentPose = getState().Pose;

        ChassisSpeeds speeds = snapController.calculate(
                currentPose,
                targetPose,
                0.0,
                targetPose.getRotation());

    // snapToPose computes chassis speeds with a HolonomicDriveController and
    // immediately applies the request using applyRequest. This is intended for
    // short, precise alignment maneuvers (e.g. final snap for AutoBuilder
    // .andThen(...).until(...)). It does not return a command — it directly
    // issues the SwerveRequest to the drivetrain.
    applyRequest(() -> new SwerveRequest.FieldCentric()
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public boolean nearPose(Pose2d pose, double translationTolerance, double rotationToleranceDeg) {

        Pose2d currentPose = getState().Pose;

        double distance = currentPose.getTranslation().getDistance(
                pose.getTranslation());

        double rotationError = Math.abs(currentPose.getRotation()
                .minus(pose.getRotation()).getDegrees());

        // Returns true when the robot pose is within the provided translation
        // and rotation tolerances. Used by auto commands to determine if a
        // snap/align step is complete.
        return distance <= translationTolerance && rotationError <= rotationToleranceDeg;
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

    // TODO implement pose get with photon camera
    public Pose2d getPose() {
        return getState().Pose;
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
