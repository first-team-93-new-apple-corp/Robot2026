package frc.robot.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class TwoStickDrive {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController OperatorController;

    public TwoStickDrive(int LeftPort, int RightPort, int opPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        OperatorController = new CommandXboxController(opPort);
    }

    // @Override 
    // public Trigger Seed() {
    //     return LeftStick.button(12);
    // }
}
