
public class ShooterMath {
    // https://www.analyzemath.com/stepbystep_mathworksheets/parabola/parabola_3_points.html
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
    
    public static double calculateV(double theta, double xfinal, double yfinal, double gravity) {
        return (xfinal / (Math.cos(theta) * Math.sqrt((2 / gravity) * (yfinal - (xfinal * Math.tan(theta))))));
    }
    public static double calculateV(double theta, double xfinal, double yfinal, double gravity,double robotX, double robotZ, double angleToHub) {
        double totalShootingVelocity = calculateV(theta,xfinal,yfinal,gravity);
        double robotRelX = calculateRobotRelX(angleToHub, robotX, robotZ);
        return totalShootingVelocity - robotRelX;
    }
    public static double calculateAdjustment(double robotX, double robotZ, double shooterVelocity, double angleToHub) {
        double robotRelZ = calculateRobotRelZ(angleToHub, robotX, robotZ);
        double adjustment = Math.PI/2 - Math.atan(shooterVelocity/robotRelZ);
        return adjustment;
    }
    public static double calculateRobotRelX(double angleToHub,double robotX,double robotZ) {
        return robotX*Math.cos(angleToHub)+robotZ*Math.sin(angleToHub);
    }
    public static double calculateRobotRelZ(double angleToHub,double robotX,double robotZ) {
        return -robotX*Math.sin(angleToHub)+robotZ*Math.cos(angleToHub);
    }
    public static double angleToAlign(double robotX, double hubX, double robotY, double hubY, double currAngle) {
        double angleToHub = 0;
        if (robotX > hubX) {
            angleToHub = Math.PI+Math.atan(Math.abs(hubY - robotY) / Math.abs(hubX - robotX));
        }
 
        angleToHub =  Math.atan(Math.abs(hubY - robotY) / Math.abs(hubX - robotX)); // Angle our robot needs to face to be algined with the hub
        // angle our robot needs to change to have its velocity z in line with the plane to the hub
        if (robotX>hubX && robotY>hubY) {
            return angleToHub - (Math.PI/2);
        }
        if (robotX<hubX && robotY>hubY) {
            return angleToHub-(Math.PI/2);
        }
        if (robotX>hubX && robotY<hubY) {
            return angleToHub - (Math.PI/2);
        }
        if (robotX<hubX && robotY<hubY) {
            return (Math.PI/2) - angleToHub;
        }
        return 1;
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
    

    
}

