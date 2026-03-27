// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        addPeriodic(() -> m_robotContainer.subsystems.questNav().commands.quest(), Milliseconds.of(20));
        addPeriodic(() -> m_robotContainer.subsystems.questNav().commands.pi(), Milliseconds.of(100));

        addPeriodic(() -> m_robotContainer.subsystems.shooter().commands.smartDashboard(), Milliseconds.of(250), Milliseconds.of(5));
        addPeriodic(() -> m_robotContainer.subsystems.shooter().commands.logging(), Milliseconds.of(250), Milliseconds.of(10));

        addPeriodic(() -> m_robotContainer.subsystems.intake().commands.smartDashboard(), Milliseconds.of(250), Milliseconds.of(5));
        addPeriodic(() -> m_robotContainer.subsystems.intake().commands.logging(), Milliseconds.of(250), Milliseconds.of(10));

        addPeriodic(() -> m_robotContainer.subsystems.manipulation().commands.logging(), Milliseconds.of(250), Milliseconds.of(10));

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        
        if (Utils.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
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
