package frc.robot.Controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class TwoStickDrive implements ControllerSchemeIO {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController operatorController;

    public TwoStickDrive(int LeftPort, int RightPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
    }

    public double deadzone(double value) {
        if (Math.abs(value) < Constants.Controls.Deadzone) {
            return 0.0;
        }
        return value;
    }

    @Override
    public double InputLeft() {
        return deadzone(-LeftStick.getY());
    }

    @Override
    public double InputUp() {
        return deadzone(-LeftStick.getX());
    }

    @Override
    public double InputTheta() {
        return deadzone(-RightStick.getX());
    }

    @Override
    public Translation2d POV() {
        return AngleToPOV(LeftStick.getHID().getPOV());
    }

    @Override
    public Trigger Seed() {
        return LeftStick.button(12);
    }
   
    @Override
    public Trigger Brake()  {
        return RightStick.trigger();
    }
    
    @Override
    public Trigger Menu() {
        throw new UnsupportedOperationException("Not available on this control scheme!");
    }

    @Override
    public Trigger Back() {
        throw new UnsupportedOperationException("Not available on this control scheme!");
    }

    @Override
    public Trigger A() {
        
        throw new UnsupportedOperationException("Not available on this control scheme!");
    }

    @Override
    public Trigger B() {
        
        throw new UnsupportedOperationException("Not available on this control scheme!");
    }

    @Override
    public Trigger X() {
        
        throw new UnsupportedOperationException("Not available on this control scheme!");
    }

    @Override
    public Trigger Y() {
        
        throw new UnsupportedOperationException("Not available on this control scheme!");
    }

}