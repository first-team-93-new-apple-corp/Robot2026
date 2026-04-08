package frc.robot.Subsystems.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants.preset;
import frc.robot.util.ShootingData;
import frc.robot.util.subsystems;

public class AutoTracker extends SequentialCommandGroup {
    private subsystems subsystems;
    private final List<Pose2d> previewPoses = new ArrayList<>();
    private final List<Pose2d> previewWaypoints = new ArrayList<>();

    public AutoTracker(subsystems subsystems, Pose2d startPose) {
        this.subsystems = subsystems;
        previewWaypoints.add(startPose);
    }

    public AutoTracker(subsystems subsystems) {
        this(subsystems, new Pose2d());
    }

    private double getDrivePoseX() {
        return subsystems.drivetrain().getState().Pose.getX();
    }

    private double getDrivePoseY() {
        return subsystems.drivetrain().getState().Pose.getY();
    }

    private double getDriveSpeedX() {
        return subsystems.drivetrain().getState().Speeds.vxMetersPerSecond;
    }

    private double getDriveSpeedY() {
        return subsystems.drivetrain().getState().Speeds.vyMetersPerSecond;
    }

    /**
     * Loads a path and adds commands to intake as the robot drives to the first
     * point in the path.
     * 
     * @param pathName
     */

    public void addIntakePath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            rememberPreview(path);
            Intake(path);
        } catch (Exception e) {
            // e.printStackTrace();
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
                rememberPreview(path);
                Intake(path);
            } catch (Exception a) {
            //    a.printStackTrace();
                DriverStation.reportError("[AutoTracker] Failed to load path: " + pathName, false);
            }
        }
    }

    /**
     * This is Deprecated use 
     * {@link #addIntakePath(String)}
     */
    @Deprecated
    public void addIntakeChoreo(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
            rememberPreview(path);
            Intake(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    /**
     * 
     * @param path
     *                 The arm deploys and intakes as the robot drives to the first
     *                 point in the path.
     */
    public void Intake(PathPlannerPath path) {
        addCommands(subsystems.Intake());
        addCommands(AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints).andThen(Commands.runOnce(() -> DogLog.timestamp("INTAKE " + path.name))));
        // addCommands(AutoBuilder.followPath(path));
    }
    
    
    public void addShootPath(String pathName, preset preset) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            rememberPreview(path);
            followSnapShoot(path, preset);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void addShootPath(String pathName, preset preset, Time delay) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            rememberPreview(path);
            followSnapShoot(path, preset, delay);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    // public void addShootPathCenter(String pathName) {
    //     try {
    //         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    //         followSnapShoot(path);
    //     } catch (Exception e) {
    //         e.printStackTrace();
    //     }
    // }

    // public void addShootPathCenterSide(String pathName) {
    //     try {
    //         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    //         followSnapShootSide(path);
    //     } catch (Exception e) {
    //         e.printStackTrace();
    //     }
    // }

    public void shootWhilstGoingTo(Supplier<Pose2d> pose) {
        Command followPath = AutoBuilder.pathfindToPose(pose.get(), AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());
        addCommands(
                followPath
                        .andThen(subsystems.PrimeHubClose())
                        .andThen(delayedShoot));
    }

    public void shootWhilstGoingToAndSnap(Pose2d pose) {
        Command followPath = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());

        addCommands(
                followPath
                        .alongWith(subsystems.PrimeHubClose())
                        .alongWith(delayedShoot)
                        .andThen(subsystems.shootFalse())
                        .andThen(
                                Commands.run(() -> subsystems.drivetrain().snapToPose(pose))
                                        .withDeadline(delayedShoot)));
    }

    public void shootWhilstFollowing(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());
        addCommands(
                followPath
                        .alongWith(subsystems.PrimeHubClose())
                        .alongWith(delayedShoot));
    }

    public void pathAndThenShoot(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());
        addCommands(
                followPath
                        .alongWith(subsystems.PrimeHubClose())
                        .andThen(delayedShoot));
    }

    public void shootWhilstFollowingAndSnap(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());

        addCommands(
                followPath
                        .alongWith(subsystems.PrimeHubClose())
                        .alongWith(delayedShoot)
                        .andThen(subsystems.shootFalse())
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(2)));
    }

    public void followSnapShoot(PathPlannerPath path, preset point) {
        Command shootPreset = subsystems.shooter().commands.velocityAndHood(point::hoodAngle, point::velocity);
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = (subsystems.shoot().alongWith(subsystems.intake().commands.wigglePivot(Seconds.of(6))));
        addCommands(
                followPath
                        .alongWith(shootPreset)
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(.5))
                        .andThen(delayedShoot).andThen(Commands.runOnce(() -> DogLog.timestamp("SHOOT " + path.name))));
                
    }
    public void followSnapShoot(PathPlannerPath path, preset point, Time delay) {
        Command shootPreset = subsystems.shooter().commands.velocityAndHood(point::hoodAngle, point::velocity);
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = (subsystems.shoot().alongWith(subsystems.intake().commands.wigglePivot(delay)));

        addCommands(
                followPath
                        .alongWith(shootPreset)
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(.5))
                        .andThen(delayedShoot).andThen(Commands.runOnce(() -> DogLog.timestamp("SHOOT " + path.name))));
                
    }

    public void goToAndThenShootAuto(Supplier<Pose2d> pose) {
        Command driveCmd = AutoBuilder.pathfindToPose(pose.get(), AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(1.25)
                .andThen(subsystems.shoot());

        addCommands(driveCmd.andThen(subsystems.PrimeHubClose().andThen(delayedShoot)));
    }

    public void addOverBump(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
            rememberPreview(path);
            overBump(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void overBump(PathPlannerPath path) {
        // addCommands(AutoBuilder.pathfindThenFollowPath(path, PathConstraints.unlimitedConstraints(12)).until(()->subsystems.drivetrain().getState().Speeds.vxMetersPerSecond < 0.2));
        addCommands(AutoBuilder.pathfindThenFollowPath(path, PathConstraints.unlimitedConstraints(12)).andThen(Commands.runOnce(() -> DogLog.timestamp("OVER BUMP " + path.name))));
        //TODO Never got to test this
        // addCommands(AutoBuilder.pathfindToPose(path.getStartingHolonomicPose().get(), PathConstraints.unlimitedConstraints(12), MetersPerSecond.of(1)));
        // addCommands(AutoBuilder.followPath(path));+
        // addCommands(Commands.runOnce(() -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
        //         .withTimeout(2));
    }

   

    public ShootingData getShootingData() {
        return subsystems.shooter().getShootingData(getDrivePoseX(), getDrivePoseY(), getDriveSpeedX(),
                getDriveSpeedY());
    }

    private void rememberPreview(PathPlannerPath path) {
        List<Pose2d> pathPoses = path.getPathPoses();
        if (pathPoses.isEmpty()) {
            return;
        }

        if (previewPoses.isEmpty()) {
            previewWaypoints.clear();
            previewWaypoints.add(pathPoses.get(0));
        } else if (!previewPoses.get(previewPoses.size() - 1).equals(pathPoses.get(0))) {
            previewWaypoints.add(pathPoses.get(0));
        }

        previewPoses.addAll(pathPoses);
        previewWaypoints.add(pathPoses.get(pathPoses.size() - 1));
    }

    public List<Pose2d> getPreviewPoses() {
        return List.copyOf(previewPoses);
    }

    public List<Pose2d> getPreviewWaypoints() {
        return List.copyOf(previewWaypoints);
    }

    public void endAuto() {
        addCommands(subsystems.IntakeFalse());
        addCommands(subsystems.PrimeFalse());
        addCommands(subsystems.shootFalse());
        addCommands(subsystems.OutakeFalse());
        addCommands(subsystems.manipulation().commands.offCommand());
    }
}
