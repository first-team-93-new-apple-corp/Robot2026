package frc.robot.util;

import static edu.wpi.first.units.Units.*;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.auto.AutoConstants;
import frc.robot.Constants;
public class ShooterMath {
    // https://www.analyzemath.com/stepbystep_mathworksheets/parabola/parabola_3_points.html
    // Static hood angle
    public static double calculateAngle(double xinit, double yinit , double xmid, double ymid, double xfinal, double yfinal) {

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
    public static double calculateAngle(double xinit, double yinit , double xmid, double ymid, double xfinal, double yfinal,double robotX,double robotZ,double poseX, double poseY, double hubX, double hubY, double hubHeight, double gravity, double angleToHub) {
        double originalTheta = calculateAngle(xinit, yinit, xmid, ymid, xfinal, yfinal);
        double totalShootingVelocity = calculateV(originalTheta,poseX, poseY, hubX,hubY,hubHeight,gravity);
        // double robotRelX = calculateRobotRelX(angleToHub, robotX, robotZ,poseX,poseY,hubX,hubY);
        double robotRelX = robotX;

        // System.out.println("Original theta " + originalTheta);
        // System.out.println("Total shooting Velocity " + totalShootingVelocity);
        // System.out.println("robotRelX");
        return Math.atan((totalShootingVelocity*Math.sin(originalTheta)/(totalShootingVelocity*Math.cos(originalTheta)-robotRelX)));
    }
    
    // Static shooting velocity calculation
    public static double calculateV(double theta, double poseX, double poseY, double hubX, double hubY,double hubHeight, double gravity) {
        double distance = Math.sqrt(Math.pow(hubX-poseX,2)+Math.pow(hubY-poseY,2));

        double xfinal = Math.abs(distance);
        double yfinal = Math.abs(hubHeight);
        return (xfinal / (Math.cos(theta) * Math.sqrt((2 / gravity) * (yfinal - (xfinal * Math.tan(theta))))));
    }
    // Shooting velocity calculation while moving
    public static double calculateV(double theta, double poseX, double poseY, double hubX, double hubY, double hubHeight, double gravity,double robotX, double robotZ, double angleToHub) {
        double totalShootingVelocity = calculateV(theta,poseX, poseY, hubX,hubY,hubHeight,gravity);
        // double robotRelX = calculateRobotRelX(angleToHub, robotX, robotZ,poseX,poseY,hubX,hubY);
        double robotRelX = robotX;
        return Math.sqrt(Math.pow(totalShootingVelocity*Math.cos(theta)-robotRelX,2)+Math.pow(totalShootingVelocity*Math.sin(theta), 2));
    }
    // Calculates angle to align to the hub (static and while moving)
    public static double calculateAdjustment(double robotX, double robotZ, double shooterVelocity, double angleToHub, double shooterPitch) {
        // double robotRelZ = calculateRobotRelZ(angleToHub, robotX, robotZ);
        double robotRelZ = robotZ;
        
        if (robotRelZ != 0) {
            double adjustment = Math.PI/2 - Math.atan2(shooterVelocity*Math.cos(shooterPitch), robotRelZ);
            // System.out.println("Adjustment " + Radians.of(adjustment).in(Degrees));
            // System.out.println("Shooter velocity " + shooterVelocity);
            // System.out.println("Shooter pitch " + Radians.of(shooterPitch).in(Degrees));
            // System.out.println("Robot rel z " + robotRelZ);
            // System.out.println("Field X " + robotX);
            // System.out.println("Field Z " + robotZ);
            return angleToHub-adjustment;
        }
        return angleToHub;
        
    }
    // Calculates robot's velocity in the x direction (towards the hub)
    public static double calculateRobotRelX(double angleToHub,double robotX,double robotZ,double poseX, double poseY, double hubX, double hubY) {
        if (poseX>hubX && poseY>hubY) {
            robotX = -robotX;
            robotZ = -robotZ;
        }
        if (poseX<hubX && poseY>hubY) {
            robotX = -robotX;
            robotZ = robotZ;
        }
        if (poseX>hubX && poseY<hubY) {
            robotX = robotX;
            robotZ = -robotZ;
        }
        if (poseX<hubX && poseY<hubY) {
            robotX = robotX;
            robotZ = robotZ;
        }
        return robotX*Math.sin(angleToHub)+robotZ*Math.cos(angleToHub);
    }
    // Calculates robots velocity in the z direction (Orthogonal to direction towards hub)
    public static double calculateRobotRelZ(double angleToHub,double robotX,double robotZ) {
        return robotX * Math.sin(Math.PI/2-angleToHub) + robotZ * Math.cos(Math.PI/2-angleToHub);
        // return -robotX*Math.sin(angleToHub)+robotZ*Math.cos(angleToHub);
        // return Math.cos(angleToHub) * robotX + Math.sin(angleToHub) * robotZ;
    }

    public static double angleToAlign(double robotX, double hubX, double robotY, double hubY) {
        double angleToHub = 0;
        angleToHub = Math.atan2((hubY - robotY) , (hubX - robotX));
 
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

    public static double[][] generateTotalDeterminant(double[] point1, double[] point2, double[] point3) { // point is a [x,y]
        // double
        double[][] matrix = {
                { Math.pow(point1[0], 2), point1[0], 1 },
                { Math.pow(point2[0], 2), point2[0], 1 },
                { Math.pow(point3[0], 2), point3[0], 1 },
        };
        return matrix;
    }

    public static double[][] generatePartialDeterminant(String partial, double[] point1, double[] point2, double[] point3) { // partial                                                                                                             // c
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
    public static ShootingData generateRotation2d(double poseX, double poseY, double velX, double velY) {
        double hubX = AutoConstants.Hub.Blue.getX();
        double hubY = AutoConstants.Hub.Blue.getY();
        double alignAngle = ShooterMath.angleToAlign(poseX, hubX, poseY, hubY);
        
        double hubHeight = 2;
        double distance = Math.sqrt(Math.pow(hubX-poseX,2)+Math.pow(hubY-poseY,2));
        double shooter_angle = ShooterMath.calculateAngle(0, 0,  distance-0.5, hubHeight+0.5  , distance, hubHeight);

        double shooter_velocity = ShooterMath.calculateV(shooter_angle,poseX,poseY,hubX,hubY,hubHeight,-9.8);
        // double nshooter_angle = ShooterMath.calculateAngle(0, 0, distance-0.5,  hubY+0.5, distance, hubY, robotX, robotZ, poseX, poseY, hubX, hubY, hubHeight, -9.8, alignAngle);
        // double nshooter_velocity = ShooterMath.calculateV(nshooter_angle,  poseX, poseY,hubX,hubY,hubHeight, -9.8, robotX, robotZ, alignAngle);     
        AngularVelocity rpm = speedToMotorRotations(shooter_velocity);
        // double alignAngleMoving = ShooterMath.calculateAdjustment(velX, velY, shooter_velocity, alignAngle,shooter_angle);
        
        Angle driveTrainAngle = Radians.of(alignAngle);
        // Angle driveTrainAngleWhileMoving = Radians.of(alignAngleMoving);
        // if (Math.abs(robotRelZ)>0.5) {
        //     System.out.println("Angle of drivetrain " + driveTrainAngle.in(Degrees));
        //     // System.out.println("Speed to shoot at " + shooter_velocity);
        //     System.out.println("Angle to shoot at " + shooter_angle);
        //     // System.out.println("Pose of robot " + poseX + " " + poseY);
        //     // System.out.println("Pose of hub " + hubX + " " + hubY);

        //     System.out.println("Robot rel z: " + robotRelZ);
        //     System.out.println("Field rel x: " + robotX);
        //     System.out.println("Field rel y: " + robotZ);
        // }
        

        return new ShootingData(new Rotation2d(driveTrainAngle), Radians.of(shooter_angle), rpm);
    }
    public static AngularVelocity speedToMotorRotations(double velocity) { // In rpm
        return RotationsPerSecond.of((velocity)/(Math.PI*Units.inchesToMeters(Constants.ShooterConstants.ShooterMotorConfigs.flyWheelDiameter.magnitude())));
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