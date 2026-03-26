package frc.robot.Subsystems.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.auto.AutoConstants.PresetShootingPoint;
import frc.robot.util.ShootingData;
import frc.robot.util.subsystems;

public class AutoTracker extends SequentialCommandGroup {
    private subsystems subsystems;
    private HashMap<PresetShootingPoint, Command> points;
    public AutoTracker(subsystems subsystems, Pose2d startPose) {
        this.subsystems = subsystems;
        points = new HashMap<>();
        points.put(AutoConstants.PresetShootingPoints.getClose(), subsystems.PrimeHubClose());
        points.put(AutoConstants.PresetShootingPoints.getCloseSide(), subsystems.PrimeHubCloseSide());
        points.put(AutoConstants.PresetShootingPoints.getFar(), subsystems.PrimeHubFar());
        points.put(null, subsystems.PrimeHubFar());
        // points.put(AutoConstants.PresetShootingPoints.getCloseSide(), subsystems.PrimeHubCloseSide());
        
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
            Intake(path);
        } catch (Exception e) {
            e.printStackTrace();
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
                Intake(path);
            } catch (Exception a) {
               a.printStackTrace();
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
        addCommands(AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints));
        // addCommands(AutoBuilder.followPath(path));
    }
    
    
    public void addShootPath(String pathName, PresetShootingPoint preset) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            followSnapShoot(path, preset);
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
                        .andThen(subsystems.Prime())
                        .andThen(delayedShoot));
    }

    public void shootWhilstGoingToAndSnap(Pose2d pose) {
        Command followPath = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());

        addCommands(
                followPath
                        .alongWith(subsystems.Prime())
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
                        .alongWith(subsystems.Prime())
                        .alongWith(delayedShoot));
    }

    public void pathAndThenShoot(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());
        addCommands(
                followPath
                        .alongWith(subsystems.Prime())
                        .andThen(delayedShoot));
    }

    public void shootWhilstFollowingAndSnap(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.75)
                .andThen(subsystems.shoot());

        addCommands(
                followPath
                        .alongWith(subsystems.AutoPrime())
                        .alongWith(delayedShoot)
                        .andThen(subsystems.shootFalse())
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(2)));
    }

    public void followSnapShoot(PathPlannerPath path, PresetShootingPoint point) {
        Command shootPreset = points.get(point);
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.5)
                .andThen(subsystems.shoot().alongWith(subsystems.intake().commands.wigglePivot(Seconds.of(6))));

        addCommands(
                followPath
                        .alongWith(shootPreset)
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(1))
                        .andThen(delayedShoot));
                
    }

    public void goToAndThenShootAuto(Supplier<Pose2d> pose) {
        Command driveCmd = AutoBuilder.pathfindToPose(pose.get(), AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(1.25)
                .andThen(subsystems.shoot());

        addCommands(driveCmd.andThen(subsystems.AutoPrime().andThen(delayedShoot)));
    }

    public void addOverBumpLeft(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
            overBump(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void addOverBumpRight(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
            overBump(path.mirrorPath());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void overBump(PathPlannerPath path) {
        // addCommands(AutoBuilder.pathfindThenFollowPath(path, PathConstraints.unlimitedConstraints(12)));
        addCommands(AutoBuilder.pathfindToPose(path.getStartingHolonomicPose().get(), PathConstraints.unlimitedConstraints(12), MetersPerSecond.of(1)));
        addCommands(AutoBuilder.followPath(path));
        // addCommands(Commands.runOnce(() -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
        //         .withTimeout(3));
    }

   

    public ShootingData getShootingData() {
        return subsystems.shooter().getShootingData(getDrivePoseX(), getDrivePoseY(), getDriveSpeedX(),
                getDriveSpeedY());
    }

    public void endAuto() {
        addCommands(subsystems.IntakeFalse());
        addCommands(subsystems.PrimeFalse());
        addCommands(subsystems.shootFalse());
        addCommands(subsystems.OutakeFalse());
        addCommands(subsystems.manipulation().commands.offCommand());
    }
}
