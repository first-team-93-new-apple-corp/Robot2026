package frc.robot.generated;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * The Interface that provides all the controlles of our robot
 *
 */
public interface ControllerSchemeIO {

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static double POVDistance = .45;
    public static double POVDistanceDiagonal = Math.sqrt(2 * (Math.pow(POVDistance, 2)));
    public static Translation2d[] POVs = {
            new Translation2d(0, 0), // Default
            new Translation2d(POVDistance, 0), // Up 1
            new Translation2d(POVDistanceDiagonal, -POVDistanceDiagonal), // up right 2
            new Translation2d(0, -POVDistance), // Right 3
            new Translation2d(-POVDistanceDiagonal, -POVDistanceDiagonal), // down right 4
            new Translation2d(-POVDistance, 0), // Down 5
            new Translation2d(-POVDistanceDiagonal, POVDistanceDiagonal), // down left 6
            new Translation2d(0, POVDistance), // left 7
            new Translation2d(POVDistanceDiagonal, POVDistanceDiagonal), // up left 8

    };

    public double InputLeft();

    public double InputUp();

    public double InputTheta();

    public Translation2d POV();

    public Trigger Seed();

    public Trigger Brake();

    public Trigger Menu();

    public Trigger Back();

    public Trigger A();

    public Trigger B();

    public Trigger X();

    public Trigger Y();


    public default double DriveLeft() {
        return InputLeft();
    }

    public default double DriveUp() {
        return InputUp();
    }

    public default double DriveTheta() {
        return InputTheta();
    }

    public default ChassisSpeeds Speeds() {
        return new ChassisSpeeds(DriveLeft(), DriveUp(), DriveTheta());
    }

    public default Translation2d AngleToPOV(int Angle) {
        switch (Angle) {
            case 0:
                return POVs[1];
            case 45:
                return POVs[2];
            case 90:
                return POVs[3];
            case 135:
                return POVs[4];
            case 180:
                return POVs[5];
            case 225:
                return POVs[6];
            case 270:
                return POVs[7];
            case 315:
                return POVs[8];
            default:
                return POVs[0];
        }
    }
}