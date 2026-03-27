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
        return halfSpeeds().getAsBoolean() ? halfLeft() : DriveLeft() * Speedthrottle();
    }

    @Override
    public double InputUp() {
        return halfSpeeds().getAsBoolean() ? halfUp() : DriveUp() * Speedthrottle();
    }

    @Override
    public double InputTheta() {
        return halfSpeeds().getAsBoolean() ? halfRotate() : DriveTheta() * Speedthrottle();
    }
}