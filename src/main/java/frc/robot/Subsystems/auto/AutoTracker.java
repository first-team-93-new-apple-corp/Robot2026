package frc.robot.Subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
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
        SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone) // Add a
                                                                                                   // 10%                                                                  // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
        driveFacingAngle.HeadingController.setPID(Constants.Drivetrain.HeadingController.kP,
                Constants.Drivetrain.HeadingController.kI, Constants.Drivetrain.HeadingController.kD);

        Command revShooter = subsystems.shooter().commands.autoShoot(getShootingData().shooterVelocity()); // TODO Implement shooting
        ParallelCommandGroup parrallel = revShooter.alongWith(Commands.waitSeconds(1));
        addCommands(parrallel.andThen(manipOuttake()));
        Command alignFunction = subsystems.drivetrain().commands.applyRequest(
                () -> driveFacingAngle.withTargetDirection(getShootingData().drivetrainAngle()));
        
        ParallelCommandGroup followShootingPathThingy = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints)
                .alongWith(alignFunction);
        addCommands(followShootingPathThingy);
    }

    public void shootWhilstFollowing(PathPlannerPath path) {
        Command alignCmd = subsystems.drivetrain().commands.applyRequest(
                () -> AutoConstants.driveFacingAngle
                        .withTargetDirection(getShootingData().drivetrainAngle()));
        Command pathFollowCmd = AutoBuilder.pathfindThenFollowPath(path, AutoConstants.constraints);
        Command hoodCmd = subsystems.shooter().commands
                .autoAngle(getShootingData().shooterAngle());
        Command velocityCmd = subsystems.shooter().commands
                .autoShoot(getShootingData().shooterVelocity());
        Command masterShootCmd = alignCmd.alongWith(velocityCmd).alongWith(hoodCmd);
        ParallelRaceGroup raceCmd = pathFollowCmd.raceWith(masterShootCmd.repeatedly());
        addCommands(raceCmd);
    }

    public ShootingData getShootingData() {
        return subsystems.shooter().getShootingData(getDrivePoseX(), getDrivePoseY(), getDriveSpeedX(),
                getDriveSpeedY());
    }
}
