// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    //         .withTimestampReplay()
    //         .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        addPeriodic(()->piPeriodic(), Milliseconds.of(250));
        addPeriodic(()->logging(), Milliseconds.of(50));
        addPeriodic(()-> smartDashboard(), Milliseconds.of(100));

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        
        if (Utils.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        questPeriodic();
    }

    public void piPeriodic(){
        m_robotContainer.subsystems.vision().piPeriodic();
    }

    public void questPeriodic(){
        m_robotContainer.subsystems.vision().questPeriodic();
    }

    public void smartDashboard(){
        m_robotContainer.subsystems.vision().smartDash();
        m_robotContainer.subsystems.intake().smartDash();
        m_robotContainer.subsystems.shooter().smartDash();
        m_robotContainer.subsystems.pds().smartDash();
    }

    public void logging(){
        m_robotContainer.subsystems.intake().log();
        m_robotContainer.subsystems.shooter().log();
        m_robotContainer.subsystems.manipulation().log();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationPeriodic() {

    }
}
