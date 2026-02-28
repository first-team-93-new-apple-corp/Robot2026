package frc.robot;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

//import com.pathplanner.lib.path.PathConstraints;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public class Drivetrain {

        public static final int FL_Drive = 1;
        public static final int FR_Drive = 2;
        public static final int BL_Drive = 3;
        public static final int BR_Drive = 4;

        public static final int FL_Steer = 5;
        public static final int FR_Steer = 6;
        public static final int BL_Steer = 7;
        public static final int BR_Steer = 8;

        public static final int FL_Cancoder = 21;
        public static final int FR_Cancoder = 22;
        public static final int BL_Cancoder = 23;
        public static final int BR_Cancoder = 24;

    }
    public class CAN {
        public static final int climber = 25;
        public static final int intakeRoller = 9;
        public static final int intakePivot = 10;
        public static final int intakePivotEncoder = 11;
        public static final int topLeftShooter = 12;
        public static final int bottomLeftShooter = 13;
        public static final int topRightShooter = 14;
        public static final int bottomRightShooter = 15;
        public static final int hoodEncoder = 16;
        public static final int hoodMotor = 17;
        public static final int kicker = 18;
        public static final int manipRoller = 19;
        public static final int manipIndexer = 20;
    }

    public class Thrustmaster {
        public static final int Trigger = 1;
        public static final int Center_Button = 2;
        public static final int Left_Button = 3;
        public static final int Right_Button = 4;
        public static final double Deadzone = 0.05;

        public class Left_Buttons {
            public static final int Top_Left = 11;
            public static final int Top_Middle = 12;
            public static final int Top_Right = 13;
            public static final int Bottom_Left = 16;
            public static final int Bottom_Middle = 15;
            public static final int Bottom_Right = 14;
        }

        public class Right_Buttons {
            public static final int Top_Left = 7;
            public static final int Top_Middle = 6;
            public static final int Top_Right = 5;
            public static final int Bottom_Left = 8;
            public static final int Bottom_Middle = 9;
            public static final int Bottom_Right = 10;
        }

        public class Axis {
            public static final int y = 1;
            public static final int x = 0;
            public static final int rotate = 2;
            public static final int slider = 3;
        }
    }

    public class F310_D {
        public static final int X = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int LeftTrigger = 7;
        public static final int RightTrigger = 8;
        public static final int Back = 9;
        public static final int Start = 10;
        public static final int LeftStick = 11;
        public static final int RightStick = 12;

        public class Axis {
            public static final int POV_Y = 0;
            public static final int POV_X = 1;
            public static final int Left_Stick_Y = 0;
            public static final int Left_Stick_X = 1;
            public static final int Right_Stick_Y = 3;
            public static final int Right_Stick_X = 2;
        }
    }

    public class F310_X {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int Back = 7;
        public static final int Start = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;

        public class Axis {
            public static final int POV_Y = 1;
            public static final int POV_X = 0;
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int Left_Stick_Y = 1;
            public static final int Left_Stick_X = 0;
            public static final int Right_Stick_Y = 5;
            public static final int Right_Stick_X = 4;
        }
    }

    public class xbox {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int Window = 7;
        public static final int Menu = 8;
        public static final int LeftPaddle = 9;
        public static final int RightPaddle = 10;

        public class Axis {
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int Left_Stick_Y = 1;
            public static final int Left_Stick_X = 0;
            public static final int Right_Stick_Y = 5;
            public static final int Right_Stick_X = 4;
        }
    }

    public class ClimberConstants {
        public static final double climberSpeed = .3;
        public static final double barHeight = -40;
        public static final double baseHeight = 0;
        public static final double kP = 0.84;
        public static final double kI = 0.15;
        public static final double kD = 0.4;
    }

    public class IntakeConstants {
        // Rollers
        public static final double intakeSpeed = 1.0;
        public static final double outtakeSpeed = -1.0;
        public static final double idleSpeed = 0.2;
        // Pivot
        public static final double pivotkP = 0.1;
        public static final double pivotkI = 0.0;
        public static final double pivotkD = 0.0;
        public static final double pivotkV = 0.0;
        public static final double pivotkA = 0.0;
        public static final double pivotkG = 0.02;
        public static final double pivotkS = 0.0;
        public static final double pivotUpSpeed = 0.2;
        public static final double pivotDownSpeed = -0.2;
        public static final Angle pivotUpPosition = Degrees.of(0);
        public static final Angle pivotDownPosition = Degrees.of(135);
        public static final Angle pivotMiddlePosition = pivotUpPosition.div(2.0);
        public static final Angle encoderOffset = Rotations.of("Replace me with encoderValue at 0 Degrees".length());
        public static final double gearBoxRatio = 9; // 9:1 for torque
        public static final double chainRatio = 1; // 1:1
    } 

    public static class ManipulationConstants {
        public static final int bottomRollerMotorID = 41; // Bottom Rollers
        public static final int sideRollerMotorID = 42; // Side Rollers
        public static final int kickerMotorID = 43; // Top Rollers

        public static class intake {
            public static final double bottomRollerSpeed = 1.0;
            public static final double sideRollerSpeed = 1.0;
            public static final double kickerSpeed = 1.0;
        }

        public static class outtake {
            public static final double bottomRollerSpeed = 1.0;
            public static final double sideRollerSpeed = 1.0;
            public static final double kickerSpeed = 1.0;
        }

        public static class idle {
            public static final double bottomRollerSpeed = 1.0;
            public static final double sideRollerSpeed = 1.0;
            public static final double kickerSpeed = 1.0;
        }
    }
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
        public static final Distance QuestX = Inches.of(15.5);
        public static final Distance QuestY = Inches.of(-1.75);
        public static final Distance QuestZ = Inches.of(20.5);

        public static final Angle QuestYawOffset = Degrees.of(2.0);
        public static final Angle QuestPitchOffset = Degrees.of(-1.5);
        public static final Angle QuestRollOffset = Degrees.of(3.0);

        public static Transform3d RobotToQuest3D = new Transform3d(QuestX.in(Meters), QuestY.in(Meters),
                QuestZ.in(Meters), new Rotation3d(QuestRollOffset, QuestPitchOffset, QuestYawOffset));
        public static Transform2d RobotToQuest2D = new Transform2d(QuestX.in(Meters), QuestY.in(Meters), new Rotation2d(QuestYawOffset.in(Degrees)));

        // public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.01, 0.01, 0.035);
        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0, 0, 0);
    }

    public class Auto {
        public static final record AutoSector(Pose2d initPose, Pose2d finalPose) {
        }

        public static final PathConstraints pathConstraints = new PathConstraints(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(0.5), RadiansPerSecond.of(Math.PI), RadiansPerSecondPerSecond.of(Math.PI/2));
    }
}