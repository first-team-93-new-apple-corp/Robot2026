package frc.robot.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class TwoStickDrive {

    public CommandJoystick LeftStick;
    public CommandJoystick RightStick;
    public CommandXboxController OperatorController;

    public TwoStickDrive(int LeftPort, int RightPort, int opPort) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        OperatorController = new CommandXboxController(opPort);
    }

}
