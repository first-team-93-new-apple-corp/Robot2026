package frc.robot;

import edu.wpi.first.units.measure.*;
import frc.robot.util.ShooterMath;
import frc.robot.util.ShootingMap;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

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
        public static final int hoodLimitSwitch = 8;
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
            public static final double kS = 0;
            public static final double kV = 0.125;
            public static final double kA = 0.02;
            public static final double kP = 0.8;
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

            public static final Map<Ranges, ShootingMap> map = Map.ofEntries(
                Map.entry(Ranges.Range1to1pt5, new ShootingMap(Radians.of(1.256),RotationsPerSecond.of(30.0))), //30
                Map.entry(Ranges.Range1pt5to2, new ShootingMap(Radians.of(1.207),RotationsPerSecond.of(31.0))), //31
                Map.entry(Ranges.Range2to2pt5, new ShootingMap(Radians.of(1.170),RotationsPerSecond.of(34.0))), //34
                Map.entry(Ranges.Range2pt5to3, new ShootingMap(Radians.of(1.15),RotationsPerSecond.of(36.0))), // 36
                Map.entry(Ranges.Range3to3pt5, new ShootingMap(Radians.of(1.12),RotationsPerSecond.of(42.0))), // 42
                Map.entry(Ranges.Range3pt5to4, new ShootingMap(Radians.of(1.11),RotationsPerSecond.of(49.0))), // 49 Allows to shoot from tench
                Map.entry(Ranges.Range4to4pt5, new ShootingMap(Radians.of(0.8),RotationsPerSecond.of(60.0))),
                Map.entry(Ranges.Range4pt5to5, new ShootingMap(Radians.of(.8),RotationsPerSecond.of(70.0))),
                Map.entry(Ranges.Range5to5pt5, new ShootingMap(Radians.of(0.8),RotationsPerSecond.of(80.0))),
                Map.entry(Ranges.Range5pt5to6, new ShootingMap(Radians.of(0.8),RotationsPerSecond.of(90.0))),
                Map.entry(Ranges.Range6to6pt5, new ShootingMap(Radians.of(1.06),RotationsPerSecond.of(100.0))),
                Map.entry(Ranges.Range6pt5to7, new ShootingMap(Radians.of(1.05),RotationsPerSecond.of(130.0))),
                Map.entry(Ranges.Range7to7pt5, new ShootingMap(Radians.of(1.05),RotationsPerSecond.of(20.0))),
                Map.entry(Ranges.Range7pt5to8, new ShootingMap(Radians.of(1.05),RotationsPerSecond.of(20.0)))
            );
            
            public static final Map<Integer,Ranges> labels = Map.ofEntries( // Labeling each segment
                Map.entry(0, Ranges.Range1to1pt5),
                Map.entry(1, Ranges.Range1pt5to2),
                Map.entry(2, Ranges.Range2to2pt5),
                Map.entry(3, Ranges.Range2pt5to3),
                Map.entry(4, Ranges.Range3to3pt5),
                Map.entry(5, Ranges.Range3pt5to4), 
                Map.entry(6, Ranges.Range4to4pt5),
                Map.entry(7, Ranges.Range4pt5to5),
                Map.entry(8, Ranges.Range5to5pt5),
                Map.entry(9, Ranges.Range5pt5to6),
                Map.entry(10, Ranges.Range6to6pt5),
                Map.entry(11, Ranges.Range6pt5to7),
                Map.entry(12, Ranges.Range7to7pt5),
                Map.entry(13, Ranges.Range7pt5to8)
            );
            public enum Ranges {
                // Each enum stores the numeric value previously used in the map (lower bound)
                Range1to1pt5,
                Range1pt5to2,
                Range2to2pt5,
                Range2pt5to3,
                Range3to3pt5,
                Range3pt5to4,
                Range4to4pt5,
                Range4pt5to5,
                Range5to5pt5,
                Range5pt5to6,
                Range6to6pt5,
                Range6pt5to7,
                Range7to7pt5,
                Range7pt5to8
            }

            
        }
        public record preset(AngularVelocity velocity, Angle hoodAngle) {
                public preset(AngularVelocity velocity) {
                    this(velocity, Degrees.of(20));
                }
            }
        public class Presets {
            private static final double closeRPS = 31;
            private static final double closeSideRPS = 33;
            private static final double inFrontOfClimbRPS = 42;
            private static final double leftWingRPS = 37.3;
            private static final double rightWingRPS = 29.8;
            private static final double depotLeftRPS = 41.6;
            private static final double depotRightRPS = 39.0;
            private static final double bumpRPS = 34.0;

            // private static final double closeDeg = 10.0;
            // private static final double closeSideDeg = 10.0;
            // private static final double inFrontOfClimbDeg = 25.0;
            // private static final double leftWingDeg = 26.4;
            // private static final double rightWingDeg = 26.4;
            // private static final double depotLeftDeg = 30.4;
            // private static final double depotRightDeg = 28.1;
            // private static final double bumpDeg = 28.0;

            private static final double closeDeg = 10.0;
            private static final double closeSideDeg = 10.0;
            private static final double inFrontOfClimbDeg = 10.0;
            private static final double leftWingDeg = 10.0;
            private static final double rightWingDeg = 10.0;
            private static final double depotLeftDeg = 10.0;
            private static final double depotRightDeg = 10.0;
            private static final double bumpDeg = 10.0;

            public static final preset close = new preset(RotationsPerSecond.of(closeRPS), Degrees.of(closeDeg));
            public static final preset closeSide = new preset(RotationsPerSecond.of(closeSideRPS),
                    Degrees.of(closeSideDeg));
            public static final preset inFrontOfClimb = new preset(RotationsPerSecond.of(inFrontOfClimbRPS),
                    Degrees.of(inFrontOfClimbDeg));
            public static final preset leftWing = new preset(RotationsPerSecond.of(leftWingRPS),
                    Degrees.of(leftWingDeg));
            public static final preset rightWing = new preset(RotationsPerSecond.of(rightWingRPS),
                    Degrees.of(rightWingDeg));
            public static final preset depotLeft = new preset(RotationsPerSecond.of(depotLeftRPS),
                    Degrees.of(depotLeftDeg));
            public static final preset depotRight = new preset(RotationsPerSecond.of(depotRightRPS),
                    Degrees.of(depotRightDeg));
            public static final preset bump = new preset(RotationsPerSecond.of(bumpRPS), Degrees.of(bumpDeg));
        }

        public class HoodMotorConfigs {
            public static final double StatorLimit = 60.0;
            public static final double SupplyLimit = 40.0;
            public static final boolean StatorLimitEnable = true;
            public static final boolean SupplyLimitEnable = true;
            public static final double kS = 0.35;
            public static final double kV = 0.15;
            public static final double kA = 0.15;
            public static final double kP = 100;
            public static final double kI = 35;
            public static final double kD = 0;
            public static final double gearRatio = 2.0; // Reduction
            public static final Angle minAngle = Degrees.of(25);
            public static final Angle maxAngle = Degrees.of(42);
            public static final Angle minAngleNoOffset = Degrees.of(0);
            public static final Angle maxAngleNoOffset = Degrees.of(15);
            public static final Angle offsetAngle = Degrees.of(25);

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