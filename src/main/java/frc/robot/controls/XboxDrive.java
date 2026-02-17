package frc.robot.controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class XboxDrive implements ControllerSchemeIO {

    public CommandXboxController Xbox;

    public XboxDrive(int port) {
        Xbox = new CommandXboxController(port);
    }

    public double deadzone(double value) {
        if (Math.abs(value) < Constants.Controls.Deadzone) {
            return 0.0;
        }
        return value;
    }

    @Override
    public double InputLeft() {
        return deadzone(-Xbox.getLeftY()) * Constants.Swerve.MaxSpeed;
    }

    @Override
    public double InputUp() {
        return deadzone(-Xbox.getLeftX()) * Constants.Swerve.MaxSpeed;
    }

    @Override
    public double InputTheta() {
        return deadzone(-Xbox.getRightX()) * Constants.Swerve.MaxAngularRate;
    }


    public Trigger climbDown(){
        return Xbox.rightStick();
    }

    public Trigger climbUp(){
        return Xbox.leftStick();
    }

    @Override
    public Translation2d POV() {
        switch (Xbox.getHID().getPOV()) {
            case 0:
                return POVs[1];
            case 90:
                return POVs[3];
            case 180:
                return POVs[5];
            case 270:
                return POVs[7];
            default:
                return POVs[0];
        }
    }

    @Override
    public Trigger Seed() {
        return Xbox.leftBumper();
    }

    @Override
    public Trigger Brake() {
        return Xbox.rightBumper();
    }

    @Override
    public Trigger Menu() {
        return Xbox.start();
    }
    @Override
    public Trigger Back() {
        return Xbox.back();
    }
    @Override
    public Trigger X() {
        return Xbox.x();
    }
    @Override
    public Trigger Y() {
        return Xbox.y();
    }
    @Override
    public Trigger A() {
        return Xbox.a();
    }
    @Override
    public Trigger B() {
        return Xbox.b();
    }

    @Override
    public Trigger Align() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Align'");
    }

   
}