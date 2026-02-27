package frc.robot.Controls;

public class ThrottleableDrive extends TwoStickDriveXboxOp{
    public ThrottleableDrive(int LeftPort, int RightPort, int opPort){
        super(LeftPort, RightPort, opPort);
    }
    private double Speedthrottle(){
        return 1-LeftStick.getThrottle();
    }
    @Override
    public double InputLeft() {
        return deadzone(-LeftStick.getY() )* Speedthrottle();
    }

    @Override
    public double InputUp() {
        return deadzone(-LeftStick.getX() ) * Speedthrottle();
    }

    @Override
    public double InputTheta() {
        return deadzone(-RightStick.getX() ) * Speedthrottle();
    }
}