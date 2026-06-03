package frc.robot.Controls;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DemoDrive extends ThrottleableDrive{
    public DemoDrive(int LeftStick, int RightStick, int op){
        super(LeftStick, RightStick, op);
    }
    private double throt = 0;
    @Override
    public double throttle(Supplier<Double> d){
        throt = d.get() <= 0.1 ? 0.1 : d.get();
        return throt;
    }

    @Override
    public double InputLeft() {
        return DriveLeft() * throt;
    }

    @Override
    public double InputUp() {
        return DriveUp() * throt;
    }

    @Override
    public double InputTheta() {
        return DriveTheta() * throt;
    }
}
