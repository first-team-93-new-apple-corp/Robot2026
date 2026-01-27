package frc.robot;

public class ShooterMath {
    // https://www.analyzemath.com/stepbystep_mathworksheets/parabola/parabola_3_points.html
    public double calculateAngle(double xinit, double xfinal, double xmid, double yint, double yfinal, double ymid){
        // double a = 
    }
    public double calculateV(double theta, double xfinal, double yfinal,double gravity) {
        return (xfinal/(Math.cos(theta)*Math.sqrt((2/gravity)*(yfinal-(xfinal*Math.tan(theta))))));
    }
    public double angleToAlign(double x1, double hubX, double y1, double hubY, double currAngle) {
        return currAngle - Math.atan(Math.abs(hubX-y1)/Math.abs(hubY-x1));
    }
    public double calculateDeterminant(double[][] matrix) {
        double a = matrix[0][0];
        double b = matrix[0][1];
        double c = matrix[0][2];
        double p = matrix[1][0];
        double q = matrix[1][1];
        double r = matrix[1][2];
        double x = matrix[2][0];
        double y = matrix[2][1];
        double z = matrix[2][2];

        return a*q*z+b*r*x+c*p*y-a*r*y-b*p*z-c*q*x;
    }
    public double calculateCoefficent(double[][] singleDeterminant, double[][] totalDeterminant) {
        return calculateDeterminant(singleDeterminant)/calculateDeterminant(totalDeterminant);
    }
    public double calculateParabola()

    return a
}
