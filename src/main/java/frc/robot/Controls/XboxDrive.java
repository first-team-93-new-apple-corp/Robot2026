package frc.robot.Controls;

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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Intake'");
    }

    @Override
    public Trigger Outtake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Outtake'");
    }

    @Override
    public Trigger Shoot() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Shoot'");
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autoRetractClimber'");
    }

    @Override
    public Trigger autoExtendClimber() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autoExtendClimber'");
    }

    @Override
    public Trigger WiggleIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'WiggleIntake'");
    }

    @Override
    public Trigger LowerIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'LowerIntake'");
    }

    @Override
    public Trigger RaiseIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'RaiseIntake'");
    }

    @Override
    public Trigger baseIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'baseIntake'");
    }

    @Override
    public Trigger maxIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'maxIntake'");
    }

    @Override
    public Trigger middleIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'middleIntake'");
    }

    @Override
    public Trigger alignShooter() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'alignShooter'");
    }

    @Override
    public Trigger seed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'seed'");
    }

    @Override
    public Trigger brake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'brake'");
    }

    @Override
    public Trigger robotRel() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'robotRel'");
    }

    @Override
    public Trigger resetClimberEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetClimberEncoder'");
    }
}