package frc.robot.Subsystems.auto;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingData;
import frc.robot.util.subsystems;

public class AutoTracker extends SequentialCommandGroup {
    private subsystems subsystems;

    public AutoTracker(subsystems subsystems, Pose2d startPose) {
        this.subsystems = subsystems;
    }

    public AutoTracker(subsystems subsystems) {
        this(subsystems, new Pose2d());
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

    private Command wigglePivot(Time time) {
        return (subsystems.intake().commands.wigglePivot(time));
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

    public void addGroundIntakePath(String pathName, LinearVelocity startSpeed, LinearVelocity endSpeed) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            groundIntake(path, startSpeed, endSpeed);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Loads a path and adds commands to intake as the robot drives to the first
     * point in the path.
     * 
     * @param pathName
     * @param endSpeed
     */
    public void addIntakePath(String pathName, LinearVelocity endSpeed) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Intake(path, endSpeed);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * 
     * @param path
     *                 The arm deploys and intakes as the robot drives to the first
     *                 point in the path.
     * @param endSpeed
     *                 The goal speed at the end of the path.
     */
    public void Intake(PathPlannerPath path, LinearVelocity endSpeed) {
        addCommands(subsystems.Intake());
        // Command cmdHooperFix = subsystems.intake().commands.autoPivotUp().andThen(subsystems.Intake());
        // addCommands(AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints).alongWith(cmdHooperFix));
        addCommands(AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints));
        // addCommands(AutoBuilder.followPath(path));
    }

    /**
     * Intakes while following the path, then stops intaking and pivots up at the
     * end of the path.
     * 
     * @param path
     * @param startSpeed
     * @param endSpeed
     */
    public void groundIntake(PathPlannerPath path, LinearVelocity startSpeed, LinearVelocity endSpeed) {
        // Find the first point in the path and drive to it while intaking, then lower
        // the arm and continue intaking as we follow the path, then stop intaking and
        // pivot up at the end of the path.
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints,
                startSpeed));
        addCommands(subsystems.Intake());
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        ParallelCommandGroup parrallel = followPath.alongWith(subsystems.Intake());
        addCommands(parrallel);
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints,
                endSpeed));
    }

    public void addShootPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            followSnapShoot(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void addShootPathCenter(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            followSnapShoot(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void addShootPathCenterSide(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            followSnapShootSide(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

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

    public void followSnapShoot(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.5)
                .andThen(subsystems.shoot().alongWith(subsystems.intake().commands.wigglePivot(Seconds.of(6))));

        addCommands(
                followPath
                        .alongWith(subsystems.PrimeHubClose())
                        // .alongWith(delayedShoot)
                        // .andThen(subsystems.shootFalse())
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(1))
                        .andThen(delayedShoot));
    }

    public void followSnapShootSide(PathPlannerPath path) {
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);

        Command delayedShoot = Commands.waitSeconds(0.5)
                .andThen(subsystems.shoot().alongWith(subsystems.intake().commands.wigglePivot(Seconds.of(6))));

        addCommands(
                followPath
                        .alongWith(subsystems.PrimeHubCloseSide())
                        // .alongWith(delayedShoot)
                        // .andThen(subsystems.shootFalse())
                        .andThen(
                                Commands.run(
                                        () -> subsystems.drivetrain().snapToPose(AutoConstants.getLastPoseInPath(path)))
                                        .withTimeout(1))
                        .andThen(delayedShoot));
    }

    public void goToAndThenShootClose(Supplier<Pose2d> pose) {
        Pose2d target = new Pose2d(pose.get().getX(), pose.get().getY() + 1, pose.get().getRotation());
        // Add a timeout so the drive command can't block the sequence indefinitely
        // during debugging. If the path controller never reports 'finished' due to
        // odometry or controller issues, the timeout ensures the auto sequence
        // continues and allows priming/shooting to run. Adjust timeout as needed.
        Command driveCmd = AutoBuilder.pathfindToPose(target, AutoConstants.constraints).withTimeout(4.0);

        Command delayedShoot = Commands.waitSeconds(1.25)
                .andThen(subsystems.shoot());

        addCommands(Commands.print("*****************go to and then shoot close to hub******************"));
        // Drive, then prime for close-hub shooting, then shoot. Timeout ensures we
        // don't block forever.
        addCommands(driveCmd.andThen(subsystems.PrimeHubClose().andThen(delayedShoot)));
        addCommands(Commands.print("*********************Ending go to and then shoot close ***************"));
    }

    public void goToAndThenShootAuto(Supplier<Pose2d> pose) {
        Command driveCmd = AutoBuilder.pathfindToPose(pose.get(), AutoConstants.constraints);
        Command delayedShoot = Commands.waitSeconds(1.25)
                .andThen(subsystems.shoot());

        addCommands(driveCmd.andThen(subsystems.AutoPrime().andThen(delayedShoot)));
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
