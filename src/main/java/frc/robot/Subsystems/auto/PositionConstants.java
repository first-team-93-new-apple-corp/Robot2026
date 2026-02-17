package frc.robot.Subsystems.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class PositionConstants {
    public static Rotation2d awayFromAlliance = Rotation2d.kZero;
    public static Rotation2d towardAlliance = Rotation2d.k180deg;

    public class HumanPlayerIntake {
        public static final Pose2d Blue = new Pose2d(0.5, 0.65, new Rotation2d());
        public static final Pose2d Red = FlippingUtil.flipFieldPose(Blue);
    }

    public class Hub {
        public static final Pose2d Blue = new Pose2d(4.625, 4.04, Rotation2d.fromDegrees(0));
        public static final Pose2d Red = FlippingUtil.flipFieldPose(Blue);
        public static final Pose3d Blue3d = new Pose3d(4.625, 4.04, 2.5, new Rotation3d());
        public static final Pose3d Red3d = new Pose3d(Red.getX(), Red.getY(), 2.5, new Rotation3d());
    }

    public static Pose2d getLastPoseInPath(PathPlannerPath path) {
        PathPoint point = path.getAllPathPoints().get(path.getAllPathPoints().size() - 1);
        return new Pose2d(point.position, point.rotationTarget.rotation());
    }

    public class startingPoses {

        private static final double startingLineBlue = 7.2;
        public static final Pose2d leftBlue = new Pose2d(startingLineBlue, 7.05, towardAlliance);
        public static final Pose2d rightBlue = new Pose2d(startingLineBlue, 0.475, towardAlliance);
        public static final Pose2d leftRed = FlippingUtil.flipFieldPose(leftBlue);
        public static final Pose2d rightRed = FlippingUtil.flipFieldPose(rightBlue);
        public static final Pose2d Center = new Pose2d(7.2, 4, towardAlliance);
    }
}