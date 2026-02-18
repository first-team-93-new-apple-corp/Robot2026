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

    public Command Intake() {
        return(subsystems.intake().commands.intake());
    }

    public Command Outtake() {
        return(subsystems.intake().commands.outtake());
    }

    public Command Idle() {
        return(subsystems.intake().commands.idle());
    }

    public Command wigglePivot() {
        return(subsystems.intake().commands.wigglePivot());
    }

    public Command pivotUp() {
        return(subsystems.intake().commands.autoPivotUp());
    }

    public Command pivotDown() {
        return(subsystems.intake().commands.autoPivotDown());
    }

    public Command pivotMiddle() {
        return(subsystems.intake().commands.autoPivotUp());
    }

    public Command manipIdle() {
        return subsystems.manipulation().commands.idleCommand();
    }

    public Command manipIntake() {
        return(subsystems.manipulation().commands.intakeCommand());
    }

    public Command manipOuttake() {
        return(subsystems.manipulation().commands.outtakeCommand());
    }

    public void addIntakingPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            addCommands(bigIntake(path));
        } catch (Exception e) {
            e.printStackTrace();
        } 
    }
    public Command bigIntake(PathPlannerPath path){
        Command cmd = null;
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        ParallelCommandGroup parrallel = followPath.alongWith(manipIdle().alongWith(Intake()));
        return cmd;
    }
}
