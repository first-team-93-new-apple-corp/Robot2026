package frc.robot.Controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class TwoStickDriveXboxOp2 extends XboxDrive2 {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController operatorController;

    public TwoStickDriveXboxOp2(int LeftPort, int RightPort, int opPort) {
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
        return CombBTN().getAsBoolean() ? deadzone(-operatorController.getLeftY()) : (halfSpeeds().getAsBoolean() ? deadzone(-LeftStick.getY()) * 0.5 : deadzone(-LeftStick.getY()));
    }

    @Override
    public double InputUp() {
        return CombBTN().getAsBoolean() ? deadzone(-operatorController.getLeftX()) : (halfSpeeds().getAsBoolean() ? deadzone(-LeftStick.getX()) * 0.5 : deadzone(-LeftStick.getX()));
    }

    @Override
    public double InputTheta() {
        return CombBTN().getAsBoolean() ? deadzone(-operatorController.getRightX()) : (halfSpeeds().getAsBoolean() ? deadzone(-RightStick.getX()) * 0.5 : deadzone(-RightStick.getX()));
    }   

    @Override
    public Trigger brake() {
        return LeftStick.button(Constants.Thrustmaster.Center_Button);
    }

    @Override
    public Trigger robotRel() {
        return LeftStick.button(Constants.Thrustmaster.Trigger);
    }

    @Override
    public Trigger Prime(){
        return operatorController.y();
    }
    @Override
    public Trigger DriverPrime() {
        return RightStick.button(Constants.Thrustmaster.Center_Button);
    }

    @Override
    public Trigger WiggleAndKick(){
        return super.WiggleAndKick();
    }

    @Override
    public Trigger Shoot(){
        return RightStick.button(Constants.Thrustmaster.Trigger).or(operatorController.rightBumper());
    }

    @Override
    public Trigger seed(){
        return LeftStick.button(Constants.Thrustmaster.Left_Buttons.Top_Middle);
    }

    @Override 
    public Trigger halfSpeeds(){
        return LeftStick.button(Constants.Thrustmaster.Right_Button).or(RightStick.button(Constants.Thrustmaster.Left_Button));
    }
    @Override 
    public Trigger resetPose() {
        return LeftStick.button(3);
    }

    @Override
    public Trigger outtakeFloorAndKickerOnly(){
        return super.outtakeFloorAndKickerOnly();
    }

}