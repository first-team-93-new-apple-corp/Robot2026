package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import org.opencv.video.Video;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Thrustmaster;


public class LEDSubsystem extends SubsystemBase{

    DigitalOutput green;
    DigitalOutput red;
    DigitalOutput blue;
    // DigitalOutput green1;
    // DigitalOutput red1;
    // DigitalOutput blue1;
    
     public void startup() {
        // if (red == null) {
            red = new DigitalOutput(0);
            green = new DigitalOutput(4);
            blue = new DigitalOutput(2);
            // setColor(Color.kBlack);
            turnLEDSOff();
        // }
    }

    public void setColor(Color m_color) {
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(1-m_color.red);
        green.enablePWM(1-m_color.green);
        blue.enablePWM(1-m_color.blue);
    }

    public void turnLEDSOff() {
        // setColor(Color.kBlack);
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(1);
        green.enablePWM(1);
        blue.enablePWM(1);
    }
    public void red() {
        // setColor(Color.kBlack);
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(0);
        green.enablePWM(1);
        blue.enablePWM(1);
    }
    public void green() {
        // setColor(Color.kBlack);
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(1);
        green.enablePWM(0);
        blue.enablePWM(1);
    }
    public void blue() {
        // setColor(Color.kBlack);
        red.disablePWM();
        green.disablePWM();
        blue.disablePWM();

        red.enablePWM(1);
        green.enablePWM(1);
        blue.enablePWM(0);
    }

    public Command LEDOn(Color m_Color) {
        return runOnce(() -> setColor(m_Color)).ignoringDisable(true);
    }

    public Command TestRed(){
        // return LEDOn(Color.kRed).ignoringDisable(true);
        return runOnce(()->red()).ignoringDisable(true);
    }

    public Command TestGreen(){
        // return LEDOn(Color.kGreen).ignoringDisable(true);
        return runOnce(()->green()).ignoringDisable(true);
    }

    public Command TestBlue(){
        // return LEDOn(Color.kBlue).ignoringDisable(true);
        return runOnce(()->blue()).ignoringDisable(true);
    }
    public Command TestOff(){
        return runOnce(()->turnLEDSOff()).ignoringDisable(true);
    }
}
