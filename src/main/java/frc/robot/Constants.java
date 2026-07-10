package frc.robot;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

        public class HeadingController {
            public static final double kP = 10;
            public static final double kI = 0;
            public static final double kD = 0.1;
        }
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
        public static final double barHeight = -60;
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
        public static final double pivotkP = 45;
        public static final double pivotkI = 0.5;
        public static final double pivotkD = 5;
        public static final double pivotkV = 0.0;
        public static final double pivotkA = 0.0;
        public static final double pivotkG = 0.05;
        public static final double pivotkS = 0.0;
        public static final double pivotUpSpeed = 0.2;
        public static final double pivotDownSpeed = -0.2;
        public static final Angle pivotUpPosition = Degrees.of(0);
        public static final Angle pivotDownPosition = Degrees.of(135);
        public static final Angle pivotMiddlePosition = Degrees.of(60);
        // public static final Angle encoderOffset = Rotations.of("Replace me with
        // encoderValue at 0 Degrees".length());
        public static final double gearBoxRatio = 9; // 9:1 for torque
        public static final double chainRatio = 1; // 1:1
    }

    public class ManipulationConstants {
        public static final int bottomRollerMotorID = 19; // Bottom Rollers
        public static final int sideRollerMotorID = 20; // Side Rollers
        public static final int kickerMotorID = 18; // Top Rollers

        public class intake {
            public static final double bottomRollerSpeed = 0.0;
            public static final double sideRollerSpeed = -0.1;
            public static final double kickerSpeed = 0.0;
        }

        public class shoot {
            public static final double bottomRollerSpeed = 0.5;
            public static final double sideRollerSpeed = -1;
            public static final double kickerSpeed = 1;
        }

        public class idle {
            public static final double bottomRollerSpeed = 0;
            public static final double sideRollerSpeed = -0.1;
            public static final double kickerSpeed = 0.0;
        }

        public class outtake {
            public static final double bottomRollerSpeed = -0.1;
            public static final double sideRollerSpeed = 0.3;
            public static final double kickerSpeed = -0.5;
        }
    }

    public class ShooterConstants {
        public class HeadingController {
            public static final double kP = 10;
            public static final double kI = 0.001;
            public static final double kD = 0.1;
        }

        public class ShooterMotorConfigs {
            public static final Distance flyWheelDiameter = Inches.of(4);
            public static final double StatorLimit = 200.0;
            public static final double SupplyLimit = 50.0;
            public static final boolean StatorLimitEnable = false;
            public static final boolean SupplyLimitEnable = false;
            public static final double kS = 0.05;
            public static final double kV = 0.125;
            public static final double kA = 0.4;
            public static final double kP = 0.15;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final AngularVelocity leftSpeed = RotationsPerSecond.of(30);
            public static final AngularVelocity rightSpeed = RotationsPerSecond.of(30);
            public static final double ShootToFlyGearRatio = 24 / 18;
            public static final double EfficiencyMultiplierClose = 1.275; // 1 ish
            public static final double EfficiencyMultiplierClimb = 1.6; // 2.954
            public static final double EfficiencyMultiplierTrench = 1.5; // 4.16
            // public static final double EfficiencyMultiplierDepotLeft = 1.8; // 5.08
            // public static final double EfficiencyMultiplierDepotRight = 1.4; // 3.61
            // Climber Controls moving to sticks
            // Shooter controls move to xbox

        }

        public class AutoShoot {
            public static final double hubHeightOffset = 0.5;
            public static final double hubXOffset = 0.35;
            public static final double PrimeEfficiencyFar = 1.5; // Need to tune
            public static final double PrimeEfficiencyClose = 1.43; // Need to tune
            public static final double ShooterHeight = 0.6;
            public static final double RangeThreshold = 3;

            /**
             * Array of rotational speeds corresponding to different ranges.
             * Starting at : 1m
             * Ending at: 8m
             * Every index: 0.25m
             */

            public static final AngularVelocity[] rpsByRangeIndex = {
                RotationsPerSecond.of(30.0), // 1.0 - 1.25m
                RotationsPerSecond.of(30.5), // 1.25 - 1.5m
                RotationsPerSecond.of(31.0), // 1.5 - 1.75m
                RotationsPerSecond.of(32.5), // 1.75 - 2.0m
                RotationsPerSecond.of(34.0), // 2.25 - 2.5m
                RotationsPerSecond.of(36.0), // 2.5 - 2.75m
                RotationsPerSecond.of(39.0), // 2.75 - 3.0m
                RotationsPerSecond.of(42.0), // 3.0 - 3.25m
                RotationsPerSecond.of(45.5), // 3.25 - 3.5m
                RotationsPerSecond.of(49.0), // 3.5 - 3.75m
                RotationsPerSecond.of(54.5), // 3.75 - 4.0m
                RotationsPerSecond.of(60.0), // 4.0 - 4.25m
                RotationsPerSecond.of(65.0), // 4.25 - 4.5m
                RotationsPerSecond.of(70.0), // 4.5 - 4.75m
                RotationsPerSecond.of(75.0), // 4.75 - 5.0m
                RotationsPerSecond.of(80.0), // 5.0 - 5.25m
                RotationsPerSecond.of(85.0), // 5.25 - 5.5m
                RotationsPerSecond.of(90.0), // 5.5 - 5.75m
                RotationsPerSecond.of(95.0), // 5.75 - 6.0m
                RotationsPerSecond.of(100.0), // 6.0 - 6.25m
                RotationsPerSecond.of(115.0), // 6.25 - 6.5m
                RotationsPerSecond.of(130.0), // 6.5 - 6.75m
                RotationsPerSecond.of(20.0), // 7.0 - 7.25m
                RotationsPerSecond.of(20.0) // 7.5 - 7.75m
            };

            
        }

        public class Presets {
            private static final double closeRPS = 31;
            private static final double closeSideRPS = 41.9;
            private static final double inFrontOfClimbRPS = 42;
            private static final double leftWingRPS = 37.3;
            private static final double rightWingRPS = 29.8;
            private static final double depotLeftRPS = 41.6;
            private static final double depotRightRPS = 39.0;
            private static final double bumpRPS = 34.0;

            public static AngularVelocity close = RotationsPerSecond.of(closeRPS);
            public static AngularVelocity closeSide = RotationsPerSecond.of(closeSideRPS);
            public static AngularVelocity inFrontOfClimb = RotationsPerSecond.of(inFrontOfClimbRPS);
            public static AngularVelocity leftWing = RotationsPerSecond.of(leftWingRPS);
            public static AngularVelocity rightWing = RotationsPerSecond.of(rightWingRPS);
            public static AngularVelocity depotLeft = RotationsPerSecond.of(depotLeftRPS);
            public static AngularVelocity depotRight = RotationsPerSecond.of(depotRightRPS);
            public static AngularVelocity bump = RotationsPerSecond.of(bumpRPS);
        }
    }

    public class Swerve {
        public static final int[] modules = { 0, 1, 2, 3 };
        public static final int[] steerMotors = { 1, 2, 3, 4 };
        public static final int[] driveMotors = { 5, 6, 7, 8 };
        public static final int[] canCoders = { 10, 11, 12, 13 };

        public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

        public class Auto {
            public static final double SnapkP = 10;
            public static final double SnapkI = 0.002;
            public static final double SnapkD = 0.1;
        }

    }

    public class Controls {
        public static final double Deadzone = 0.04;
    }

    public class Quest {
        public static final Distance QuestX = Inches.of(-11);
        public static final Distance QuestY = Inches.of(-9.75);
        public static final Distance QuestZ = Inches.of(17);

        public static final Angle QuestYawOffset = Degrees.of(180);
        public static final Angle QuestPitchOffset = Degrees.of(0);
        public static final Angle QuestRollOffset = Degrees.of(90);

        public static Transform3d RobotToQuest3D = new Transform3d(QuestX.in(Meters), QuestY.in(Meters),
                QuestZ.in(Meters),
                new Rotation3d(QuestRollOffset, QuestPitchOffset, QuestYawOffset));

        // public static Transform2d RobotToQuest2D = new Transform2d(QuestX.in(Meters),
        // QuestY.in(Meters),
        // new Rotation2d(QuestYawOffset.in(Degrees)));
        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0, 0, 0);
    }

    public class Photon {
        public static final Transform3d kRobotToCam = new Transform3d(
                new Translation3d(Inches.of(-11), Inches.of(-10.5), Inches.of(11.25)),
                new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(180)));

        public static final Matrix<N3, N1> singleTagDevs = VecBuilder.fill(0.2, 0.2, 0.2);
        public static final Matrix<N3, N1> multiTagDevs = VecBuilder.fill(0.02, 0.02, 0.05); // probably not used
        public static final Matrix<N3, N1> standardDevs = VecBuilder.fill(0.15, 0.15, 0.05); // probably not used

    }

    public class Auto {
        public static final record AutoSector(Pose2d initPose, Pose2d finalPose) {
        }

        public static final PathConstraints pathConstraints = new PathConstraints(MetersPerSecond.of(1.0),
                MetersPerSecondPerSecond.of(0.5), RadiansPerSecond.of(Math.PI),
                RadiansPerSecondPerSecond.of(Math.PI / 2));
    }
}
