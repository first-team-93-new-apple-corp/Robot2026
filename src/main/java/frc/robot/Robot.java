// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.LEDSubsystem;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private LEDSubsystem muy_leds;
  private final RobotContainer m_robotContainer;

  public Robot() {
    muy_leds = new LEDSubsystem();
    m_robotContainer = new RobotContainer(muy_leds);
    
  }

  @Override
  public void robotInit(){
    muy_leds.startup();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    addPeriodic(() -> m_robotContainer.visionPeriodic(), Milliseconds.of(20), Milliseconds.of(5));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
