package frc.robot.Controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class TwoStickDriveXboxOp extends XboxDrive {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController operatorController;

    public TwoStickDriveXboxOp(int LeftPort, int RightPort, int opPort) {
        super(LeftPort, RightPort, opPort);
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        operatorController = new CommandXboxController(opPort);
    }

    public double deadzone(double value) {
        if (Math.abs(value) < Constants.Thrustmaster.Deadzone) {
            return 0.0;
        }
        return value;
    }

    @Override
    public double InputLeft() {
        return halfSpeeds().getAsBoolean() ? halfLeft() : deadzone(-LeftStick.getY());
    }

    @Override
    public double InputUp() {
        return halfSpeeds().getAsBoolean() ? halfUp() : deadzone(-LeftStick.getY());
    }

    @Override
    public double InputTheta() {
        return halfSpeeds().getAsBoolean() ? halfRotate() : deadzone(-LeftStick.getY());
    }

    @Override
    public double halfLeft() {
        return InputLeft() * 0.5;
    }
    
    @Override
    public double halfUp() {
        return InputUp() * 0.5;
    }

    @Override
    public double halfRotate() {
        return InputTheta() * 0.5;
    }

    @Override
    public Trigger brake() {
        return LeftStick.button(Constants.Thrustmaster.Center_Button);
    }

    @Override
    public Trigger fieldRel() {
        return LeftStick.button(Constants.Thrustmaster.Left_Buttons.Top_Middle);
    }

    @Override
    public Trigger robotRel() {
        return LeftStick.button(Constants.Thrustmaster.Trigger);
    }

    @Override
    public Trigger primeShooter(){
        return LeftStick.button(Constants.Thrustmaster.Trigger);
    }

    @Override
    public Trigger seed(){
        return LeftStick.button(Constants.Thrustmaster.Left_Buttons.Top_Middle);
    }

    @Override 
    public Trigger halfSpeeds(){
        return LeftStick.button(Constants.Thrustmaster.Right_Button).or(RightStick.button(Constants.Thrustmaster.Left_Button));
    }
}