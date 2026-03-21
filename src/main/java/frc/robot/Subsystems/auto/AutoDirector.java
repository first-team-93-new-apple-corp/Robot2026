package frc.robot.Subsystems.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.util.subsystems;

public class AutoDirector {
    // This is the chooser that will be displayed on the dashboard to select the
    // auto.
    public final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    public final List<Auto> Autos = new ArrayList<>();
    private final subsystems autoSubsystems;
    // private final PathConstraints constraints = Constants.Auto.pathConstraints;
    private static RobotConfig config = null;

    // Auto
    private SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);
        // .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    // For quick hardware debugging you can force the path-follow consumer to use
    // open-loop voltage drive requests instead of velocity control. This is
    // commented out by default — uncomment to test whether the drivetrain
    // responds to open-loop commands coming from AutoBuilder. Remember to
    // restore the original setting for competition use.
    //
    // private ApplyRobotSpeeds autoRequest = new ApplyRobotSpeeds()
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            // .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public AutoDirector(subsystems autoSubsystems) {
        this.autoSubsystems = autoSubsystems;
        try {
            config = RobotConfig.fromGUISettings();

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

    // Configure AutoBuilder last
    //
    // Parameters passed to AutoBuilder.configure:
    // - poseSupplier: returns the current robot pose for the path controller
    // - poseReset: used to reset odometry when starting a path (important if your
    //   odometry isn't aligned with the path start)
    // - speedSupplier: returns the current measured chassis speeds (used by some
    //   controllers and logging)
    // - chassisSpeedConsumer: called each trajectory cycle with desired ChassisSpeeds
    //   The consumer is responsible for translating those speeds into a SwerveRequest
    //   and applying it to the drivetrain. In this robot we call drivetrain.setControl()
    //   with an ApplyRobotSpeeds request.
    // - controller: the PPHolonomicDriveController used to compute chassis speeds
    // - config: RobotConfig (tuning) from PathPlanner GUI
    // - allianceFlipSupplier: boolean supplier to mirror paths for the red alliance
    // - drivetrain subsystem: used to set command requirements for path commands
    AutoBuilder.configure(
        this::getPose,
        this::resetAutoPose,
        this::getSpeeds,
        // Wrap the consumer so we can log the requested speeds each cycle. This
        // helps debug when a path appears to "do nothing" — we can see whether
        // AutoBuilder is producing non-zero commands and whether they reach the
        // drivetrain consumer.
        (speeds, feedforwards) -> {
            // Per-cycle logging: chassis speeds requested by the path controller
            // Keep this lightweight; if you need to long-term log, push to NetworkTables
            System.out.printf("[AutoBuilder] vx=%.3f vy=%.3f omega=%.3f\\n",
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond);
            System.out.flush();

                    // Apply the requested speeds to the drivetrain using the pre-configured
                    // ApplyRobotSpeeds request (autoRequest.withSpeeds(...)).
                    autoSubsystems.drivetrain().setControl(autoRequest.withSpeeds(speeds));

                    // --- QUICK OPEN-LOOP TEST (commented out) ------------------------
                    // If you'd rather test open-loop drive (motors commanded with
                    // voltages) to verify the drivetrain responds, uncomment one of
                    // the options below. These are examples only — uncomment temporarily
                    // during testing and re-comment for normal autonomous runs.
                    //
                    // Option A: Swap the autoRequest declaration to OpenLoopVoltage
                    // (see the top of this file where autoRequest is declared) and
                    // keep the call above. That will cause the drivetrain to receive
                    // open-loop requests directly.
                    //
                    // Option B: Directly send an OpenLoop FieldCentric request here:
                    // autoSubsystems.drivetrain().setControl(
                    //     new SwerveRequest.FieldCentric()
                    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    //         .withVelocityX(speeds.vxMetersPerSecond)
                    //         .withVelocityY(speeds.vyMetersPerSecond)
                    //         .withRotationalRate(speeds.omegaRadiansPerSecond)
                    // );
                    // ----------------------------------------------------------------
        },
        new PPHolonomicDriveController(
            new PIDConstants(2, 0.0, 0.1),
            new PIDConstants(2, 0.0, 0.0)),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance. If true PathPlanner will mirror trajectories across the field
            // so you don't need separate paths for each alliance.
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        autoSubsystems.drivetrain() // Reference to this subsystem to set requirements
    );
        addAutos();

    }

    public static RobotConfig getRobotConfig() {
        return config;
    }

    private Pose2d getPose() {
        return autoSubsystems.drivetrain().getState().Pose;
    }

    private ChassisSpeeds getSpeeds() {
        return autoSubsystems.drivetrain().getState().Speeds;
    }

    public void resetAutoPose(Pose2d pose) {
        autoSubsystems.drivetrain().resetPose(pose);
        autoSubsystems.questNav().commands.setRobotPose(new Pose3d(pose));
    }

    // This is the basis of the Auto. It contains the name of the auto, the command
    // to run, and the initial pose.
    public record Auto(String name, Command command, Pose2d initPose) {
        // This allows us to create an Auto without specifying an initial pose,
        // defaulting to field origin.
        public Auto(String name, Command command) {
            this(name, command, new Pose2d());
        }
    }

    public Auto selection() {
        return autoChooser.getSelected();
    }

    public void addAutos() {
        autoChooser.setDefaultOption("Do Nothing", new Auto("Do Nothing", Commands.none()));
        // Autos.add(Demo());
        // Autos.add(PreloadCenter());
        // Autos.add(PreloadLeftOrRight());
        // Autos.add(PreloadClimbLeft());
        // Autos.add(PreloadClimbRight());
        Autos.add(DepotScoreCenter());
        Autos.add(DepotScoreCenterLeft());
        Autos.add(DepotScoreCenterRight());
        // Autos.add(PreloadOutpost());
        for (Auto auto : Autos) {
            autoChooser.addOption(auto.name, auto);
        }
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Auto combineAutos(Auto... autos) {
        List<Command> list = new ArrayList<>();
        for (Auto auto : autos) {
            list.add(Commands.print("Starting Auto: " + auto.name));
            list.add(auto.command);
        }
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        for (Command command : list) {
            cmds.addCommands(command);
        }
        return new Auto("Combined Auto", cmds, new Pose2d());
    }

    public Auto Demo() {
        PathPlannerPath testPath = null;
        List<Command> list = new ArrayList<>();

        try {
            testPath = PathPlannerPath.fromPathFile("Demo Path");
        } catch (Exception e) {
            e.printStackTrace();
        }
        Pose2d startPose = testPath.getStartingDifferentialPose();
        Pose2d correctedStartPose = new Pose2d(startPose.getX(), startPose.getY(), new Rotation2d());
        // Shhh definintly not doing this vvv
        // Assuming the position of the robot is correct, we tell the robot that it is
        // at the starting pose of the path
        list.add(autoSubsystems.questNav().commands.setRobotPose(new Pose3d(startPose)));
        // Shhh definintly not doing this ^^^
        list.add(Commands.print("Running Demo Auto"));
        list.add(Commands.print("****************************************** START POSE: " + startPose.toString()));
        list.add(Commands
                .print("****************************************** Better POSE: " + correctedStartPose.toString()));

        list.add(AutoBuilder.followPath(testPath));
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        for (Command command : list) {
            cmds.addCommands(command);
        }

        return new Auto("Demo", cmds, correctedStartPose);
    }

    public Alliance getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get();
        }
        return Alliance.Blue;
    }

    public Auto PreloadCenter() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        // Pose2d startPose = new Pose2d(AutoConstants.Hub.getHub().toPose2d().getX()-1,
        //         AutoConstants.Hub.getHub().toPose2d().getY(), AutoConstants.Hub.getHub().toPose2d().getRotation());

        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        tracker.addCommands(autoSubsystems.questNav().commands.setRobotPose(new Pose3d(startPose)));
        tracker.addCommands(Commands.print("*********************Starting Automos**************"));
        tracker.addCommands(Commands.print(startPose.toString()));

        if (getAlliance() == Alliance.Red) {
            tracker.goToAndThenShootClose(()->new Pose2d(autoSubsystems.drivetrain().getState().Pose.getX() + 1,
                    autoSubsystems.drivetrain().getState().Pose.getY(),
                    autoSubsystems.drivetrain().getState().Pose.getRotation().rotateBy(new Rotation2d(Degrees.of(0)))));
        } else {
            tracker.goToAndThenShootClose(()->new Pose2d(autoSubsystems.drivetrain().getState().Pose.getX() - 1,
                    autoSubsystems.drivetrain().getState().Pose.getY(),
                    autoSubsystems.drivetrain().getState().Pose.getRotation().rotateBy(new Rotation2d(Degrees.of(0)))));
        }
        tracker.endAuto();
        return new Auto("Score Preload Only", tracker, startPose);
    }

    public Auto PreloadLeftOrRight() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        if (getAlliance() == Alliance.Red) {
            tracker.goToAndThenShootClose(()->new Pose2d(autoSubsystems.drivetrain().getState().Pose.getX() + 1,
                    autoSubsystems.drivetrain().getState().Pose.getY(),
                    autoSubsystems.drivetrain().getState().Pose.getRotation().rotateBy(new Rotation2d(Degrees.of(0)))));
        } else {
            tracker.goToAndThenShootClose(()->new Pose2d(autoSubsystems.drivetrain().getState().Pose.getX() - 1,
                    autoSubsystems.drivetrain().getState().Pose.getY(),
                    autoSubsystems.drivetrain().getState().Pose.getRotation().rotateBy(new Rotation2d(Degrees.of(0)))));
        }
        return new Auto("Score Preload Only", tracker, startPose);
    }

    public Auto PreloadClimbLeft() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        tracker.addCommands(autoSubsystems.questNav().commands.setRobotPose(new Pose3d(startPose)));
        tracker.addShootPath("null");
        return new Auto("Score Preload Climb", tracker, startPose);
    }

    public Auto PreloadClimbRight() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        // tracker.addCommands(autoSubsystems.questNav().commands.setRobotPose(new
        // Pose3d(startPose)));
        // tracker.addShootPath("shootingPath");
        return new Auto("Score Preload Climb", tracker, startPose);
    }

    public Auto DepotScoreCenter() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        tracker.addIntakePath("depotIntake", MetersPerSecond.of(0.1));
        tracker.addShootPath("Shoot Center");
        tracker.endAuto();
        return new Auto("IntakeDepotScore", tracker, startPose);
    }
    public Auto DepotScoreCenterLeft() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        tracker.addIntakePath("depotIntake", MetersPerSecond.of(0.1));
        tracker.addShootPath("Shoot Center");
        tracker.endAuto();
        return new Auto("IntakeDepotScore", tracker, startPose);
    }
    public Auto DepotScoreCenterRight() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        tracker.addIntakePath("depotIntake", MetersPerSecond.of(0.1));
        tracker.addShootPath("Shoot Center Right");
        tracker.endAuto();
        return new Auto("IntakeDepotScore", tracker, startPose);
    }

    public Auto PreloadOutpost() {
        Pose2d startPose = autoSubsystems.drivetrain().getPose();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
        tracker.addIntakePath("depotIntake", MetersPerSecond.of(0.1));
        if (getAlliance() == Alliance.Red) {
            tracker.goToAndThenShootClose(()->new Pose2d(autoSubsystems.drivetrain().getState().Pose.getX() - 1,
                    autoSubsystems.drivetrain().getState().Pose.getY() - 1,
                    autoSubsystems.drivetrain().getState().Pose.getRotation().rotateBy(new Rotation2d(Degrees.of(0)))));
        } else {
            tracker.goToAndThenShootClose(()->new Pose2d(autoSubsystems.drivetrain().getState().Pose.getX() + 1,
                    autoSubsystems.drivetrain().getState().Pose.getY() + 1,
                    autoSubsystems.drivetrain().getState().Pose.getRotation().rotateBy(new Rotation2d(Degrees.of(0)))));
        }
        
        return new Auto("IntakeOutpostScore", tracker, startPose);

    }

    // public Auto PreloadDepotClimb(){
    // Pose2d startPose = autoSubsystems.drivetrain().getStartingPose();
    // AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
    // tracker.addCommands(autoSubsystems.questNav().commands.setRobotPose(new
    // Pose3d(startPose)));
    // return null;
    // }

    // public Auto PreloadOutpostCLimb(){
    // Pose2d startPose = autoSubsystems.drivetrain().getStartingPose();
    // AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);
    // tracker.addCommands(autoSubsystems.questNav().commands.setRobotPose(new
    // Pose3d(startPose)));
    // return null;
    // }

    // public Auto doNothing(){
    // return null;
    // }

    // public Auto moveForward(){
    // return null;
    // }
    /*
     * TODO
     * Score preload before intake?
     * Intake then score intake and preload?
     * What to do before climb
     * shooting positions? / path to shoot on.
     * farther away to get out of the way of other robots
     * Follow up question: timing to score
     * figure out how to implement custom delays in paths to avoid robots
     * 
     */

}
