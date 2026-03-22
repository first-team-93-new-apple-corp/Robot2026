package frc.robot.Controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.xbox;

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
        return new Trigger(() -> false);
    }

    @Override
    public Trigger manExtendClimber() {
        return new Trigger(() -> false);
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
        return Xbox.a();
    }

    @Override
    public Trigger RaiseIntake() {
        return Xbox.y();
    }

    @Override
    public Trigger middleIntake() {
        return Xbox.start();
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
        return new Trigger(() -> false);
    }

    @Override
    public Trigger resetClimberEncoder() {
        return Xbox.back();
    }

    @Override
    public double halfRotate() {
        return InputTheta() * 0.5;
    }

    @Override
    public Trigger halfSpeeds() {
        return new Trigger(() -> false);
    }

    @Override
    public Trigger PrimeLeft() {
        return Xbox.povLeft();
    }

    @Override
    public Trigger PrimeRight() {
        return new Trigger(()->false);
    }

    @Override
    public Trigger 
    PrimeClose() {
        return Xbox.povUp();
    }

    @Override
    public Trigger PrimeFar() {
        return Xbox.povDown();
    }

    @Override
    public Trigger DriverPrime() {
        return Prime();
    }

    @Override
    public Trigger testingButton(){
        return Xbox.start();
    }

    @Override
    public Trigger resetPose() {
        return Xbox.back();
    }

    @Override
    public Trigger manHood(){
        return Xbox.rightTrigger(0.01);
    }
    @Override
    public Trigger manShoot(){
        return Xbox.leftTrigger(0.01);
    }

    @Override
    public double rightTrigger(){
        return Xbox.getRightTriggerAxis();
    }

    @Override
    public double leftTrigger(){
        return Xbox.getLeftTriggerAxis();
    }

    @Override
    public Trigger Pass() {
        return Xbox.povRight();
    }


}