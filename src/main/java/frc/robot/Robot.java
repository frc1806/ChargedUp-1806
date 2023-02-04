// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.shuffleboard.ShuffleboardManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public ShuffleboardManager mShuffleboardManager;
  private RobotContainer m_robotContainer;

  public void allPeriodic(){
    mShuffleboardManager.updateAllTabs();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    mShuffleboardManager = new ShuffleboardManager();
    mShuffleboardManager.registerTabs();
    PathPlannerServer.startServer(5811);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    mShuffleboardManager.updateAllTabs();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic(){
    allPeriodic();
  }

  @Override
  public void autonomousPeriodic(){
    allPeriodic();
  }

  @Override
  public void teleopPeriodic(){
    allPeriodic();
  }

  @Override
  public void testPeriodic(){
    allPeriodic();
  }

  @Override
  public void simulationPeriodic(){
    allPeriodic();
  }
}