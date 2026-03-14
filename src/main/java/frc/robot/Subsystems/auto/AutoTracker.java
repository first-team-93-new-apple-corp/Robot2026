package frc.robot.Subsystems.auto;

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
     * Loads a path and adds commands to intake as the robot drives to the first point in the path.
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
     * The arm deploys and intakes as the robot drives to the first point in the path.
     * @param endSpeed
     * The goal speed at the end of the path.
     */
    public void Intake(PathPlannerPath path, LinearVelocity endSpeed) {
        addCommands(subsystems.Intake());
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints, endSpeed));
    }

    /**
     * Intakes while following the path, then stops intaking and pivots up at the end of the path.
     * @param path
     * @param startSpeed
     * @param endSpeed
     */
    public void groundIntake(PathPlannerPath path,  LinearVelocity startSpeed, LinearVelocity endSpeed) {
        // Find the first point in the path and drive to it while intaking, then lower the arm and continue intaking as we follow the path, then stop intaking and pivot up at the end of the path.
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints, startSpeed));
        addCommands(subsystems.Intake());
        Command followPath = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        ParallelCommandGroup parrallel = followPath.alongWith(subsystems.Intake());
        addCommands(parrallel);
        addCommands(AutoBuilder.pathfindToPose(AutoConstants.getFirstPoseInPath(path), AutoConstants.constraints, endSpeed));
    }

    public void addShootPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            shootWhilstFollowing(path);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void shootWhilstGoingTo(Pose2d pose) {
        // SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        //     .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
        //     .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone)
        //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // driveFacingAngle.HeadingController.setPID(Constants.Drivetrain.HeadingController.kP,
        //         Constants.Drivetrain.HeadingController.kI, Constants.Drivetrain.HeadingController.kD);

        // Command revShooter = subsystems.shooter().commands.autoShoot(getShootingData().shooterVelocity()); // TODO Implement shooting
        // ParallelCommandGroup parrallel = revShooter.alongWith(Commands.waitSeconds(0.5));
        // addCommands(parrallel.andThen(subsystems.shoot(3)));
        // Command alignFunction = subsystems.drivetrain().commands.applyRequest(
        //         () -> driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle()));
        
        
        ParallelCommandGroup followShootingPathThingy = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints)
                .alongWith(subsystems.shoot(999));
        addCommands(subsystems.Prime().andThen(followShootingPathThingy));
    }

    public void shootWhilstFollowing(PathPlannerPath path) {
        // Command alignCmd = subsystems.drivetrain().commands.applyRequest(
        //         () -> AutoConstants.driveFacingAngle
        //                 .withTargetDirection(getShootingData().drivetrainAngle()));
        Command pathFollowCmd = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        // Command hoodCmd = subsystems.shooter().commands
        //         .autoAngle(getShootingData().shooterAngle());
        // Command velocityCmd = subsystems.shooter().commands
        //         .autoShoot(getShootingData().shooterVelocity());
        Command masterShootCmd = subsystems.Prime();
        Command shoot = subsystems.shoot(99);
        ParallelRaceGroup raceCmd = pathFollowCmd.raceWith(shoot);
        addCommands(masterShootCmd.alongWith(Commands.waitSeconds(2)).andThen(raceCmd));
    }

    public void goToAndThenShoot(Pose2d pose) {
        Command driveCmd = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints);
        Command shootCmd = subsystems.shoot(3);
        addCommands(driveCmd.andThen(shootCmd));
    }

    public ShootingData getShootingData() {
        return subsystems.shooter().getShootingData(getDrivePoseX(), getDrivePoseY(), getDriveSpeedX(),
                getDriveSpeedY());
    }
}
