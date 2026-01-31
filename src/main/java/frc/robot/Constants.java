package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class Swerve {
        public static final int[] modules = { 0, 1, 2, 3 };
        public static final int[] steerMotors = { 1, 2, 3, 4 };
        public static final int[] driveMotors = { 5, 6, 7, 8 };
        public static final int[] canCoders = { 10, 11, 12, 13 };

        public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);
    }

    public class Controls {
        public static final double Deadzone = 0.04;
    }
    public class Quest {
        public static final Distance QuestX = Inches.of(14.5);
        public static final Distance QuestY = Inches.of(-1.75);
        public static final Distance QuestZ = Inches.of(20.5);

        public static final Angle QuestYawOffset = Degrees.of(12.0);
        public static final Angle QuestPitchOffset = Degrees.of(-1.5);
        public static final Angle QuestRollOffset = Degrees.of(3.0);

        public static Transform3d RobotToQuest = new Transform3d(
                QuestX.in(Meters),
                QuestY.in(Meters),
                QuestZ.in(Meters),
                new Rotation3d(QuestRollOffset, QuestPitchOffset, QuestYawOffset));

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
                0.01,
                0.01,
                0.035
        );
    }
}