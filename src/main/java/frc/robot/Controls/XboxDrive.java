package frc.robot.Controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class XboxDrive implements ControllerSchemeIO {

    public CommandXboxController Xbox;
    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;

    public XboxDrive(int port) {
        Xbox = new CommandXboxController(port);
    }

    public XboxDrive(int port, int port2, int opPort) {
        Xbox = new CommandXboxController(opPort);
        LeftStick = new CommandJoystick(port);
        RightStick = new CommandJoystick(port2);
    }

    public double deadzone(double value) {
        if (Math.abs(value) < Constants.Thrustmaster.Deadzone) {
            return 0.0;
        }
        return value;
    }

    @Override
    public double InputLeft() {
        return deadzone(-Xbox.getLeftY());
    }

    @Override
    public double InputUp() {
        return deadzone(-Xbox.getLeftX());
    }

    @Override
    public double InputTheta() {
        return deadzone(-Xbox.getRightX());
    }

    @Override
    public Trigger Intake() {
        return Xbox.x();
    }

    @Override
    public Trigger Outtake() {
        return Xbox.b();
    }

    @Override
    public Trigger Shoot() {
        return Xbox.rightTrigger();
    }

    @Override
    public Trigger manRetractClimber() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manRetractClimber'");
    }

    @Override
    public Trigger manExtendClimber() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manExtendClimber'");
    }

    @Override
    public Trigger autoRetractClimber() {
        return Xbox.leftStick();
    }

    @Override
    public Trigger autoExtendClimber() {
        return Xbox.rightStick();
    }

    @Override
    public Trigger WiggleIntake() {
        return Xbox.rightBumper();
    }

    @Override
    public Trigger LowerIntake() {
        return Xbox.povDown();
    }

    @Override
    public Trigger RaiseIntake() {
        return Xbox.povUp();
    }

    @Override
    public Trigger middleIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'middleIntake'");
    }

    @Override
    public Trigger Prime() {
        return Xbox.leftBumper();
    }

    @Override
    public Trigger seed() {
        return Xbox.leftBumper();
    }

    @Override
    public Trigger brake() {
        return Xbox.start();
    }

    @Override
    public Trigger robotRel() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'robotRel'");
    }

    @Override
    public Trigger resetClimberEncoder() {
        return Xbox.back();
    }

   
    public Trigger presetClimb() {
        // TODO Auto-generated method stub
        return Xbox.a();
    }

    public Trigger presetClose() {
        // TODO Auto-generated method stub
        return Xbox.y();
    }
    public Trigger presetLeft() {
        // TODO Auto-generated method stub
        return Xbox.povLeft();
    }
    public Trigger presetRight() {
        // TODO Auto-generated method stub
        return Xbox.povRight();
    }
    @Override
    public double halfRotate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'halfRotate'");
    }

    @Override
    public Trigger halfSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'halfSpeeds'");
    }

    @Override
    public Trigger PrimeLeft() {
       return Xbox.povLeft();
    }

    @Override
    public Trigger PrimeRight() {
       return Xbox.povRight();
    }

    @Override
    public Trigger PrimeClose() {
        return Xbox.povUp();
    }

    @Override
    public Trigger PrimeFar() {
        return Xbox.povDown();
    }

    


}