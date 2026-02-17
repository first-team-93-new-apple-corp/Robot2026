package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.ShooterMath;
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

    // Auto
    private SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAuto();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAuto();

    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAuto();
    }
    
    public Command alignToHub() {
        

        try{
            
            RobotConfig robotConfig = null;
            try {
                robotConfig = RobotConfig.fromGUISettings();
                
            } catch (Exception e) {
                // Handle exception as needed
                e.printStackTrace();
            }
            PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
            double poseX = getState().Pose.getX();
            double poseY = getState().Pose.getY();
            double rotation = getState().Pose.getRotation().getRadians();
            System.out.println("**************************************** " + rotation);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(poseX,poseY,getState().Pose.getRotation()),
                new Pose2d(poseX,poseY,getState().Pose.getRotation()),
                new Pose2d(poseX,poseY,getState().Pose.getRotation())
            );
            System.out.println("****************************************"+Rotation2d.fromRadians(ShooterMath.angleToAlign(poseX, Constants.Field.hubX, poseY, Constants.Field.hubY)).toString());
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    Constants.Auto.pathConstraints,
                    null, 
                    new GoalEndState(0.0, Rotation2d.fromRadians(ShooterMath.angleToAlign(poseX, Constants.Field.hubX, poseY, Constants.Field.hubY)))
            );
            double hubX = Constants.Field.hubX;
            double hubY = Constants.Field.hubY;
            double robotX = getState().Speeds.vxMetersPerSecond;
            double robotZ = getState().Speeds.vyMetersPerSecond; 
           
            double hubHeight = 4;
            double distance = Math.sqrt(Math.pow(hubX-poseX,2)+Math.pow(hubY-poseY,2));
            double alignAngle = ShooterMath.angleToAlign(poseX, hubX, poseY, hubY);
            double shooter_angle = ShooterMath.calculateAngle(0, 0,  distance-0.5, hubHeight+0.5  , distance, hubHeight);
            
            double nshooter_velocity = ShooterMath.calculateV(shooter_angle,  poseX, poseY,hubX,hubY,hubHeight, -9.8, robotX, robotZ, alignAngle);     
            double nshooter_angle = ShooterMath.calculateAngle(0, 0, distance-0.5,  hubY+0.5, distance, hubY, robotX, robotZ, poseX, poseY, hubX, hubY, hubHeight, -9.8, alignAngle);

            System.out.println("Angle of drivetrain " + Rotation2d.fromRadians(ShooterMath.angleToAlign(poseX, Constants.Field.hubX, poseY, Constants.Field.hubY)).getDegrees());
            System.out.println("Speed to shoot at " + nshooter_velocity);
            System.out.println("Angle to shoot at " + nshooter_angle);


            return new FollowPathCommand(
                    path,
                    
                    ()->getState().Pose, // Robot pose supplier
                    ()->getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> {}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    robotConfig, // The robot configuration
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    public void configureAuto() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
            
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(() -> getState().Pose, this::resetPose, () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(autoRequest.withSpeeds(speeds)),

                new PPHolonomicDriveController(new PIDConstants(0.75, 0.0, 0), new PIDConstants(0.5, 0.0, 0.0)), 
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this // Reference to this subsystem to set requirements
        );
    }
    // @Override 
    // public void ppResetPose(Pose2d pose) {
    //     this.resetPose(pose);
        
    // }
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
