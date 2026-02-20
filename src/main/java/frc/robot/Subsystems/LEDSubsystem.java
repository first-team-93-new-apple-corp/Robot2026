package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    DigitalOutput red;
    DigitalOutput green;
    DigitalOutput blue;

    double r;
    double g;
    double b;

    public void startup() {
        red = new DigitalOutput(0);
        green = new DigitalOutput(4);
        blue = new DigitalOutput(2);
        setColor(Color.kWhite);

        SmartDashboard.putNumber("LED Red", 0);
        SmartDashboard.putNumber("LED Green", 0);
        SmartDashboard.putNumber("LED Blue", 0);
    }

    public void setColor(Color m_color) {
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(m_color.red);
        green.enablePWM(m_color.green);
        blue.enablePWM(m_color.blue);
    }
    public void setColor(double r, double g, double b) {
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(r);
        green.enablePWM(g);
        blue.enablePWM(b);
    }

    public void turnLEDSOff() {
        setColor(Color.kBlack);
    }
    @Override
    public void periodic() {
        r = SmartDashboard.getNumber("LED Red", 0);
        g = SmartDashboard.getNumber("LED Green", 0);
        b = SmartDashboard.getNumber("LED Blue", 0);
        setColor(r,g,b);
    }

    public Command TestRed() {
        return runOnce(() -> setColor(Color.kRed));
    }
    public Command TestGreen() {
        return runOnce(() -> setColor(Color.kGreen));
    }
    public Command TestBlue() {
        return runOnce(() -> setColor(Color.kBlue));
    }
}