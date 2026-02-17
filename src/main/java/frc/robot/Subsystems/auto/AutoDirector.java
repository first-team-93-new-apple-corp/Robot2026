package frc.robot.Subsystems.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.subsystems;

public class AutoDirector {
    // This is the chooser that will be displayed on the dashboard to select the
    // auto.
    public final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    public final List<Auto> Autos = new ArrayList<>();
    private final subsystems autoSubsystems;
    private final PathConstraints constraints = Constants.Auto.pathConstraints;

    // Auto
    private ApplyRobotSpeeds autoRequest = new ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public AutoDirector(subsystems autoSubsystems) {
        this.autoSubsystems = autoSubsystems;
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose,
                this::resetAutoPose,
                this::getSpeeds,
                (speeds, feedforwards) -> autoSubsystems.drivetrain().setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                        new PIDConstants(0.75, 0.0, 0),
                        new PIDConstants(0.5, 0.0, 0.0)),
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
                },
                autoSubsystems.drivetrain() // Reference to this subsystem to set requirements
        );
        addAutos();
    }

    private Pose2d getPose() {
        return autoSubsystems.drivetrain().getState().Pose;
    }

    private ChassisSpeeds getSpeeds() {
        return autoSubsystems.drivetrain().getState().Speeds;
    }

    public void resetAutoPose(Pose2d pose) {
        autoSubsystems.drivetrain().resetPose(pose);
        autoSubsystems.questNav().commands.resetQuestPose(pose);
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
        Autos.add(TestAuto());
        Autos.add(Demo());
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

    public Auto TestAuto() {
        PathPlannerPath testPath = null;
        List<Command> list = new ArrayList<>();

        try {
            testPath = PathPlannerPath.fromPathFile("Many Over Bump");
        } catch (Exception e) {
            e.printStackTrace();
        }
        Pose2d startPose = testPath.getStartingDifferentialPose();
        Pose2d correctedStartPose = new Pose2d(startPose.getX(), startPose.getY(), new Rotation2d());
        // Shhh definintly not doing this vvv
        // Assuming the position of the robot is correct, we tell the robot that it is
        // at the starting pose of the path
        list.add(autoSubsystems.questNav().commands.resetQuestPose(correctedStartPose));
        // Shhh definintly not doing this ^^^
        list.add(Commands.print("Testing Auto"));
        list.add(Commands.print("****************************************** START POSE: " + startPose.toString()));
        list.add(Commands
                .print("****************************************** Better POSE: " + correctedStartPose.toString()));

        list.add(AutoBuilder.followPath(testPath));
        // After following the path, we tell the robot to pathfind back to the starting
        // pose, which should be the same as the corrected start pose
        Command cmd = AutoBuilder.pathfindToPose(correctedStartPose, constraints)
                .andThen(Commands.print("Pathfound back to start: " + correctedStartPose.toString()));
        list.add(cmd);

        SequentialCommandGroup cmds = new SequentialCommandGroup();
        for (Command command : list) {
            cmds.addCommands(command);
        }

        return new Auto("TestAuto", cmds, correctedStartPose);
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
        list.add(autoSubsystems.questNav().commands.resetQuestPose(correctedStartPose));
        // Shhh definintly not doing this ^^^
        list.add(Commands.print("Running Demo Auto"));
        list.add(Commands.print("****************************************** START POSE: " + startPose.toString()));
        list.add(Commands.print("****************************************** Better POSE: " + correctedStartPose.toString()));

        list.add(AutoBuilder.followPath(testPath));
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        for (Command command : list) {
            cmds.addCommands(command);
        }

        return new Auto("Demo", cmds, correctedStartPose);
    }

    public Auto testing(){
        Pose2d startPose = new Pose2d();
        AutoTracker tracker = new AutoTracker(autoSubsystems, startPose);

        return new Auto("Testing", tracker, startPose);
    }
}
