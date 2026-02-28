package frc.robot.Controls;

import edu.wpi.first.math.geometry.Translation2d;
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
    public Trigger brake() {
        return RightStick.button(Constants.Thrustmaster.Trigger);
    }

    @Override
    public Trigger robotRel() {
        return RightStick.button(Constants.Thrustmaster.Center_Button);
    }

    @Override
    public Trigger primeShooter(){
        return LeftStick.button(Constants.Thrustmaster.Trigger);
    }

    @Override
    public Trigger seed(){
        return LeftStick.button(Constants.Thrustmaster.Left_Buttons.Top_Middle);
    }
}