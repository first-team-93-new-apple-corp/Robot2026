package frc.robot.Subsystems.auto;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.ManipulationConstants.intake;
import frc.robot.util.subsystems;

public class AutoTracker extends SequentialCommandGroup {
    private subsystems subsystems;

    public AutoTracker(subsystems subsystems, Pose2d startPose) {
        this.subsystems = subsystems;
    }

    private Command Intake() {
        return (subsystems.intake().commands.intake());
    }

    private Command Outtake() {
        return (subsystems.intake().commands.outtake());
    }

    private Command Idle() {
        return (subsystems.intake().commands.idle());
    }

    private Command wigglePivot() {
        return (subsystems.intake().commands.wigglePivot());
    }

    private Command pivotUp() {
        return (subsystems.intake().commands.autoPivotUp());
    }

    private Command pivotDown() {
        return (subsystems.intake().commands.autoPivotDown());
    }

    private Command pivotMiddle() {
        return (subsystems.intake().commands.autoPivotUp());
    }

    private Command manipIdle() {
        return subsystems.manipulation().commands.idleCommand();
    }

    private Command manipIntake() {
        return (subsystems.manipulation().commands.intakeCommand());
    }

    private Command manipOuttake() {
        return (subsystems.manipulation().commands.outtakeCommand());
    }

    public void addGroundIntakePath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            groundIntake(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void addIntakePath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Intake(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void Intake(PathPlannerPath path) {
        double secondsDelay = 2; // Waiting for balls to drop from outpost
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints));
        addCommands(pivotDown());
        addCommands(Idle());
        addCommands(manipIdle());
    }

    public void groundIntake(PathPlannerPath path) {
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints));
        addCommands(pivotDown());
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        ParallelCommandGroup parrallel = followPath.alongWith(manipIdle().alongWith(Intake()));
        addCommands(parrallel);
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints));
    }

    public void shootWhilstGoingTo(Pose2d pose) {
        Command revShooter = Commands.none(); // TODO Implement shooting
        ParallelCommandGroup parrallel = revShooter.alongWith(Commands.waitSeconds(1));
        addCommands(parrallel.andThen(manipOuttake()));
        Command alignFunction = Commands.none(); // TODO Implement shooting aligning
        ParallelCommandGroup followShootingPathThingy = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints)
                .alongWith(alignFunction);
        addCommands(followShootingPathThingy);
    }
}
