package frc.robot.util;

import static edu.wpi.first.units.Units.*;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.auto.AutoConstants;
import frc.robot.Constants;
public class ShooterMath {
    public static double calculateAngle(double xinit, double yinit, double xmid, double ymid, double xfinal,
            double yfinal) {

        double[] point1 = { xinit, yinit };
        double[] point2 = { xmid, ymid };
        double[] point3 = { xfinal, yfinal };
        // Coefficients to parabola
        double a = calculateCoefficent(generatePartialDeterminant("a", point1, point2, point3),
                generateTotalDeterminant(point1, point2, point3));
        double b = calculateCoefficent(generatePartialDeterminant("b", point1, point2, point3),
                generateTotalDeterminant(point1, point2, point3));
        double c = calculateCoefficent(generatePartialDeterminant("c", point1, point2, point3),
                generateTotalDeterminant(point1, point2, point3));

        return Math.atan(2 * a * xinit + b);
    }

    // Calculating hood angle with movement
    public static double calculateAngle(double xinit, double yinit, double xmid, double ymid, double xfinal,
            double yfinal, double robotX, double robotZ, double poseX, double poseY, double hubX, double hubY,
            double hubHeight, double gravity, double angleToHub, double adjustment) {
        double distance = Math.sqrt(Math.pow(hubX - poseX, 2) + Math.pow(hubY - poseY, 2)); // Add this to code
        double originalTheta = calculateAngle(0, 0, distance - 0.5, hubHeight + 0.5, distance, hubHeight); // replace
                                                                                                           // this
                                                                                                           // calculate
                                                                                                           // angle in
                                                                                                           // code
        double totalShootingVelocity = calculateV(originalTheta, poseX, poseY, hubX, hubY, hubHeight, gravity);
        // double robotRelX = calculateRobotRelX(angleToHub, robotX,
        // robotZ,poseX,poseY,hubX,hubY);
        double robotRelX = robotX;
        double adj = adjustment - angleToHub;

        // System.out.println("Original theta " + originalTheta);
        // System.out.println("Total shooting Velocity " + totalShootingVelocity);
        // System.out.println("robotRelX");
        return Math.atan((totalShootingVelocity * Math.sin(originalTheta))
                / (totalShootingVelocity * Math.cos(originalTheta) / Math.cos(adj) - robotRelX));
    }

    // Static shooting velocity calculation
    public static double calculateV(double theta, double poseX, double poseY, double hubX, double hubY,
            double hubHeight, double gravity) {
        double distance = Math.sqrt(Math.pow(hubX - poseX, 2) + Math.pow(hubY - poseY, 2));

        double xfinal = Math.abs(distance);
        double yfinal = Math.abs(hubHeight);
        // System.out.println("Time " + Math.sqrt((2 / gravity) * (yfinal - (xfinal * Math.tan(theta)))));
        return (xfinal / (Math.cos(theta) * Math.sqrt((2 / gravity) * (yfinal - (xfinal * Math.tan(theta))))));
    }

    // Shooting velocity calculation while moving
    public static double calculateV(double theta, double poseX, double poseY, double hubX, double hubY,
            double hubHeight, double gravity, double robotX, double robotZ, double angleToHub, double adjustment,
            double shooterPitch) {
        double totalShootingVelocity = calculateV(theta, poseX, poseY, hubX, hubY, hubHeight, gravity);
        // double robotRelX = calculateRobotRelX(angleToHub, robotX,
        // robotZ,poseX,poseY,hubX,hubY);

        double robotRelX = robotX;
        double adj = adjustment - angleToHub;

        return (Math.sqrt(
                Math.pow((totalShootingVelocity * Math.cos(theta) - robotRelX) / Math.cos(adj), 2)
                        + Math.pow(totalShootingVelocity * Math.sin(theta), 2)));
    }

    // Calculates angle to align to the hub (static and while moving)
    public static double calculateAdjustment(double robotX, double robotZ, double shooterVelocity, double angleToHub,
            double shooterPitch) {
        // double robotRelZ = calculateRobotRelZ(angleToHub, robotX, robotZ);
        double robotRelZ = robotZ;

        if (robotRelZ != 0) {
            // double adjustment = Math.PI/2 -
            // Math.atan2(shooterVelocity*Math.cos(shooterPitch), robotRelZ);
            // double adjustment =
            // Math.asin(robotRelZ/(shooterVelocity*Math.cos(shooterPitch))); // add this to
            // robot code
            double adjustment = Math.asin(robotRelZ
                    / Math.sqrt(Math.pow(shooterVelocity * Math.cos(shooterPitch), 2) + Math.pow(robotRelZ, 2)));
            return angleToHub + adjustment;
        }
        return angleToHub;

    }

    // Calculates robot's velocity in the x direction (towards the hub)
    public static double calculateRobotRelX(double angleToHub, double robotX, double robotZ, double poseX, double poseY,
            double hubX, double hubY) {
        if (poseX > hubX && poseY > hubY) {
            robotX = -robotX;
            robotZ = -robotZ;
        }
        if (poseX < hubX && poseY > hubY) {
            robotX = -robotX;
            robotZ = robotZ;
        }
        if (poseX > hubX && poseY < hubY) {
            robotX = robotX;
            robotZ = -robotZ;
        }
        if (poseX < hubX && poseY < hubY) {
            robotX = robotX;
            robotZ = robotZ;
        }
        return robotX * Math.sin(angleToHub) + robotZ * Math.cos(angleToHub);
    }

    // Calculates robots velocity in the z direction (Orthogonal to direction
    // towards hub)
    public static double calculateRobotRelZ(double angleToHub, double robotX, double robotZ) {
        return robotX * Math.sin(Math.PI / 2 - angleToHub) + robotZ * Math.cos(Math.PI / 2 - angleToHub);
        // return -robotX*Math.sin(angleToHub)+robotZ*Math.cos(angleToHub);
        // return Math.cos(angleToHub) * robotX + Math.sin(angleToHub) * robotZ;
    }

    public static double angleToAlign(double robotX, double hubX, double robotY, double hubY) {
        double angleToHub = 0;
        angleToHub = Math.atan2((hubY - robotY), (hubX - robotX));
        // if (robotX > hubX) {
        // angleToHub = Math.PI+Math.atan((hubY - robotY) / (hubX - robotX));
        // } else {
        // angleToHub = Math.atan((hubY - robotY) / (hubX - robotX)); // Angle our robot
        // needs to face to be algined with the hub

        // }

        return angleToHub;
    }

    public static double calculateDeterminantValue(double[][] matrix) {
        double a = matrix[0][0];
        double b = matrix[0][1];
        double c = matrix[0][2];
        double p = matrix[1][0];
        double q = matrix[1][1];
        double r = matrix[1][2];
        double x = matrix[2][0];
        double y = matrix[2][1];
        double z = matrix[2][2];

        return a * q * z + b * r * x + c * p * y - a * r * y - b * p * z - c * q * x;
    }

    public static double calculateCoefficent(double[][] singleDeterminant, double[][] totalDeterminant) {
        return calculateDeterminantValue(singleDeterminant) / calculateDeterminantValue(totalDeterminant);
    }

    public static double[][] generateTotalDeterminant(double[] point1, double[] point2, double[] point3) { // point is a
                                                                                                           // [x,y]
        // double
        double[][] matrix = {
                { Math.pow(point1[0], 2), point1[0], 1 },
                { Math.pow(point2[0], 2), point2[0], 1 },
                { Math.pow(point3[0], 2), point3[0], 1 },
        };
        return matrix;
    }

    public static double[][] generatePartialDeterminant(String partial, double[] point1, double[] point2,
            double[] point3) { // partial // c
        double[][] matrix = generateTotalDeterminant(point1, point2, point3);
        if (partial.equals("a")) {
            matrix[0][0] = point1[1];
            matrix[1][0] = point2[1];
            matrix[2][0] = point3[1];
        }
        if (partial.equals("b")) {
            matrix[0][1] = point1[1];
            matrix[1][1] = point2[1];
            matrix[2][1] = point3[1];
        }
        if (partial.equals("c")) {
            matrix[0][2] = point1[1];
            matrix[1][2] = point1[1];
            matrix[2][2] = point1[1];
        }
        return matrix;
    }

    public static double[] calcShootingDataWhileMoving(double robotX, double robotZ, double shooter_velocity,
            double originalPitch, double angleToHub) {
        double vcosThetaPrime = Math
                .sqrt(Math.pow(shooter_velocity * Math.cos(originalPitch) - robotX, 2) + Math.pow(robotZ, 2));
        double thetaPrime = Math.atan2(shooter_velocity * Math.sin(originalPitch), vcosThetaPrime);
        double vPrime = vcosThetaPrime / Math.cos(thetaPrime);
        double adjustment = angleToHub + Math.asin(
                robotZ / Math.sqrt(Math.pow(shooter_velocity * Math.cos(originalPitch)-robotX, 2) + Math.pow(robotZ, 2)));
        return new double[] { thetaPrime, vPrime, adjustment }; // pitch, velocity, adjustment
    }
    public static ShootingData generateRotation2d(double poseX, double poseY, double velX, double velY) {
        double hubX = AutoConstants.Hub.getHub().getX();
        double hubY = AutoConstants.Hub.getHub().getY();
        double alignAngle = ShooterMath.angleToAlign(poseX, hubX, poseY, hubY);
        
        double hubHeight = AutoConstants.Hub.getHub().getZ();
        double distance = Math.sqrt(Math.pow(hubX-poseX,2)+Math.pow(hubY-poseY,2));
        double shooter_angle = ShooterMath.calculateAngle(0, 0,  distance-0.5, hubHeight+0.5  , distance, hubHeight);


        double shooter_velocity = ShooterMath.calculateV(shooter_angle,poseX,poseY,hubX,hubY,hubHeight,-9.8);   
        AngularVelocity rpm = speedToMotorRotations(shooter_velocity);
        Angle driveTrainAngle = Radians.of(alignAngle);

        
        double[] shootingDataWhileMoving = calcShootingDataWhileMoving(velX, velY, shooter_velocity, shooter_angle, alignAngle);
        double velocityMoving = shootingDataWhileMoving[1];
        double angleMoving = shootingDataWhileMoving[0];
        double adjustmentMoving = shootingDataWhileMoving[2];
        AngularVelocity rpmMoving = speedToMotorRotations(shooter_velocity);
        Angle driveTrainAngleMoving = Radians.of(adjustmentMoving);
        
        // Stationary shooting
        return new ShootingData(new Rotation2d(driveTrainAngle), Degrees.of(90).minus(Radians.of(shooter_angle)), rpm);
        // On the fly
        // return new ShootingData(new Rotation2d(driveTrainAngleMoving), Degrees.of(90).minus(Radians.of(angleMoving)),rpmMoving);
        
    }
    public static AngularVelocity speedToMotorRotations(double velocity) { // In rpm
        return RotationsPerSecond.of((Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierFar* (velocity)/(Math.PI*Units.inchesToMeters(Constants.ShooterConstants.ShooterMotorConfigs.flyWheelDiameter.magnitude()))));
    }
    public static AngularVelocity speedToMotorRotationsforClose(double velocity) { // In rpm
        return RotationsPerSecond.of(Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierClose* (velocity)/(Math.PI*Units.inchesToMeters(Constants.ShooterConstants.ShooterMotorConfigs.flyWheelDiameter.magnitude())));
    }
    public static double getEfficiency(Angle angle) {
        if (Constants.ShooterConstants.HoodMotorConfigs.offsetAngle.gt(Radians.of(Math.PI/2-angle.in(Radians)))) {
            return Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierClose;
        }
        return Constants.ShooterConstants.ShooterMotorConfigs.EfficiencyMultiplierFar;
    }
    
    // public static void main(String[] args) throws Exception {
    //   double hubX = 4;
    //   double hubY = 4;
    //   double robotX = 1;
    //   double robotZ = 2;
    //   double poseX = 0;
    //   double poseY = 0;
      
    //   double hubHeight = 4;
    //   double distance = Math.sqrt(Math.pow(hubX-poseX,2)+Math.pow(hubY-poseY,2));
    //   double alignAngle = angleToAlign(poseX, hubX, poseY, hubY);
    //   double shooter_angle = calculateAngle(0, 0,  distance-0.5, hubHeight+0.5  , distance, hubHeight);
    //   System.out.println("Static angle of shooter" + shooter_angle);
    //   double shooter_velocity = calculateV(shooter_angle,poseX, poseY,hubX,hubY,hubHeight, -9.8);
    //   System.out.println("Static velocity " + shooter_velocity);
    //   double nshooter_velocity = calculateV(shooter_angle,  poseX, poseY,hubX,hubY,hubHeight, -9.8, robotX, robotZ, alignAngle);
    //   System.out.println("Theoretical velocity while moving" + nshooter_velocity);
    //   double adjustment = calculateAdjustment(robotX,robotZ,nshooter_velocity,angleToAlign(poseX, hubX, poseY, hubY));
    //   System.out.println("Theoretical adjustment angle to shoot while moving " + adjustment + " Normal angle " + alignAngle) ;
    //   double nshooter_angle = calculateAngle(0, 0, distance-0.5,  hubY+0.5, distance, hubY, robotX, robotZ, poseX, poseY, hubX, hubY, hubHeight, -9.8, alignAngle);
    //   System.out.println("Theoretical hood angle to shoot from while moving " + nshooter_angle);
    // }

    
}