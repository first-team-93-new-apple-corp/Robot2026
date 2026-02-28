package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class Shooter {
        public static final Distance flyWheelDiameter = Inches.of(3.615);
    }
    public class Swerve {
        public static final int[] modules = { 0, 1, 2, 3 };
        public static final int[] steerMotors = { 1, 2, 3, 4 };
        public static final int[] driveMotors = { 5, 6, 7, 8 };
        public static final int[] canCoders = { 10, 11, 12, 13 };

        public static final double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }
    public class Field {
        // x 4.04
        // y = 0.4.625
        public static final Pose2d hub = new Pose2d(4.23,-0.45,new Rotation2d());

        public static final double hubHeight = 1.83;
    }
    public class Controls {
        public static final double Deadzone = 0.04;
    }

    public class Quest {
        public static final Distance QuestX = Inches.of(15.5);
        public static final Distance QuestY = Inches.of(-1.75);
        public static final Distance QuestZ = Inches.of(20.5);

        public static final Angle QuestYawOffset = Degrees.of(2.0);
        public static final Angle QuestPitchOffset = Degrees.of(-1.5);
        public static final Angle QuestRollOffset = Degrees.of(3.0);

        public static Transform3d RobotToQuest = new Transform3d(QuestX.in(Meters), QuestY.in(Meters),
                QuestZ.in(Meters), new Rotation3d(QuestRollOffset, QuestPitchOffset, QuestYawOffset));

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.005, 0.005, 0.017);
    }

    public class Auto {
        public static final record AutoSector(Pose2d initPose, Pose2d finalPose) {
        }

        public static final PathConstraints pathConstraints = new PathConstraints(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(0.5), RadiansPerSecond.of(Math.PI), RadiansPerSecondPerSecond.of(Math.PI/2));
        public static RobotConfig robotConfig = null;
    }
}