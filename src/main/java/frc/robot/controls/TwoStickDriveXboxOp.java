package frc.robot.controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class TwoStickDriveXboxOp implements ControllerSchemeIO {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController operatorController;

    public TwoStickDriveXboxOp(int LeftPort, int RightPort, int opPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        operatorController = new CommandXboxController(opPort);
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
    public Trigger Brake() {
        return RightStick.trigger();
    }

    @Override
    public Trigger Menu() {
        return operatorController.start();
    }

    @Override
    public Trigger Back() {
        return operatorController.back();

    }

    @Override
    public Trigger A() {
        return operatorController.a();
    }

    @Override
    public Trigger B() {
        return operatorController.b();

    }

    @Override
    public Trigger X() {
        return operatorController.x();

    }

    @Override
    public Trigger Y() {
        return operatorController.y();
    }

}