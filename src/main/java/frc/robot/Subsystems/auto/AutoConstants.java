package frc.robot.Subsystems.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.ShooterMath;

public class AutoConstants {
    public static Rotation2d awayFromAlliance = Rotation2d.kZero;
    public static Rotation2d towardAlliance = Rotation2d.k180deg;

    public class HumanPlayerIntake {
        public static final Pose2d Blue = new Pose2d(0.5, 0.65, new Rotation2d());
        public static final Pose2d Red = FlippingUtil.flipFieldPose(Blue);
    }

    public class Hub {
        public static final Pose2d Blue = new Pose2d(Inches.of(182.1), Inches.of(158.84), Rotation2d.fromDegrees(0));
        public static final Pose2d Red = FlippingUtil.flipFieldPose(Blue);
        public static final Pose3d Blue3d = new Pose3d(Inches.of(182.1), Inches.of(158.84), Inches.of(56.5), new Rotation3d());
        public static final Pose3d Red3d = new Pose3d(Red.getX(), Red.getY(), 1.47, new Rotation3d());
        public static Pose3d getHub() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return Red3d;
                } 
                return Blue3d; // Default to blue
            } else {
                return Blue3d;
            }
        }
    }
    public record PresetShootingPoint(Pose2d pose, AngularVelocity velocity, Angle hoodAngle) {}
    public class PresetShootingPoints {
        public static final PresetShootingPoint BlueClose = new PresetShootingPoint(new Pose2d(3.525, 3.965, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotationsforClose(6.4).times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierClose), Degrees.of(90).minus(Radians.of(1.395)));
        public static final PresetShootingPoint RedClose = new PresetShootingPoint(FlippingUtil.flipFieldPose(BlueClose.pose), BlueClose.velocity, BlueClose.hoodAngle);
        public static final PresetShootingPoint BlueCloseSide = new PresetShootingPoint(new Pose2d(3.525, 3.965, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotations(6.55).times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierClose), Degrees.of(90).minus(Radians.of(1.395)));
        public static final PresetShootingPoint RedCloseSide = new PresetShootingPoint(FlippingUtil.flipFieldPose(BlueCloseSide.pose), BlueCloseSide.velocity, BlueCloseSide.hoodAngle);
        public static final PresetShootingPoint BlueInfrontofClimb = new PresetShootingPoint(new Pose2d(1.599, 3.771, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotations(7.13).times(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierClimb), Degrees.of(90).minus(Radians.of(1.15)));
        public static final PresetShootingPoint RedInfrontofClimb = new PresetShootingPoint(FlippingUtil.flipFieldPose(BlueInfrontofClimb.pose), BlueInfrontofClimb.velocity, BlueInfrontofClimb.hoodAngle);
        public static final PresetShootingPoint BlueLeftWing = new PresetShootingPoint(new Pose2d(3.208, 7.304, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotations(7.45), Degrees.of(90).minus(Radians.of(1.11)));
        public static final PresetShootingPoint BlueRightWing = new PresetShootingPoint(new Pose2d(3.208, 0.766, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotationsforClose(7.45), Degrees.of(90).minus(Radians.of(1.11)));
        public static final PresetShootingPoint RedRightWing = new PresetShootingPoint(FlippingUtil.flipFieldPose(BlueLeftWing.pose), BlueLeftWing.velocity, BlueLeftWing.hoodAngle);
        public static final PresetShootingPoint RedLeftWing = new PresetShootingPoint(FlippingUtil.flipFieldPose(BlueRightWing.pose), BlueRightWing.velocity, BlueRightWing.hoodAngle);
        public static final PresetShootingPoint BlueDepotLeft = new PresetShootingPoint(new Pose2d(0.498, 6.996, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotations(8.299), Degrees.of(90).minus(Radians.of(1.04)));
        public static final PresetShootingPoint BlueDepotRight = new PresetShootingPoint(new Pose2d(0.547, 4.854, Rotation2d.fromDegrees(0)), ShooterMath.speedToMotorRotations(7.789), Degrees.of(90).minus(Radians.of(1.08)));

        public static PresetShootingPoint getClose() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return RedClose;
                } 
                return BlueClose; // Default to blue
            }
            return null;
        }

        public static PresetShootingPoint getCloseSide() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return RedCloseSide;
                } 
                return BlueCloseSide; // Default to blue
            }
            return null;
        }

        public static PresetShootingPoint getFar() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return RedInfrontofClimb;
                } 
                return BlueInfrontofClimb; // Default to blue
            }
            return null;
        }

        public static PresetShootingPoint getLeft() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return RedLeftWing;
                } 
                return BlueLeftWing; // Default to blue
            }
            return null;
        }
         public static PresetShootingPoint getRight() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return RedRightWing;
                } 
                return BlueRightWing; // Default to blue
            }
            return null;
        }
         public static PresetShootingPoint getDepotLeft() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return BlueDepotLeft;
                } 
                return BlueDepotLeft; // Default to blue
            }
            return null;
        }
         public static PresetShootingPoint getDepotRight() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return BlueDepotRight;
                } 
                return BlueDepotRight; // Default to blue
            }
            return null;
        }
        // right up agasibnst hub
        // left corner
        // right croner
    }

    public static Pose2d getLastPoseInPath(PathPlannerPath path) {
        PathPoint point = path.getAllPathPoints().get(path.getAllPathPoints().size() - 1);
        return new Pose2d(point.position, point.rotationTarget.rotation() == null ? new Rotation2d(0.0) : point.rotationTarget.rotation());
    }

    public static Pose2d getFirstPoseInPath(PathPlannerPath path) {
        PathPoint point = path.getAllPathPoints().get(0);
        // Return the initial pose for a PathPlanner path. If the rotation target
        // or its rotation is null we default to zero rotation so pathfindToPose
        // calls have a sensible heading to approach. Protect against null
        // rotationTarget to avoid NPEs when paths omit rotation targets.
        Rotation2d rot = new Rotation2d(0);
        if (point.rotationTarget != null && point.rotationTarget.rotation() != null) {
            rot = point.rotationTarget.rotation();
        }
        return new Pose2d(point.position, rot);
    }

    public class startingPoses {

        private static final double startingLineBlue = 7.2;
        public static final Pose2d leftBlue = new Pose2d(startingLineBlue, 7.05, towardAlliance);
        public static final Pose2d rightBlue = new Pose2d(startingLineBlue, 0.475, towardAlliance);
        public static final Pose2d leftRed = FlippingUtil.flipFieldPose(leftBlue);
        public static final Pose2d rightRed = FlippingUtil.flipFieldPose(rightBlue);
        public static final Pose2d Center = new Pose2d(7.2, 4, towardAlliance);
    }

    // Default path constraints used by AutoBuilder (maxSpeed, maxAcceleration, etc.)
    // Tune these values in AutoConstants if your robot is not following paths as
    // expected (too aggressive controllers can fail to converge).
    public static final PathConstraints constraints = new PathConstraints(4, 3, Math.PI*2 , Math.PI * 3);

    public static final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                  // m
}