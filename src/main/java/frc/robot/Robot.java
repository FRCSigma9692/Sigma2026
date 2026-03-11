// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot {
    boolean AutoResult = false;
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    Color testColor = new Color(255, 255, 255);

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        //SmartDashboard.putBoolean("Result Of Selection",m_robotContainer.drivetrain.wonAuto);
         SmartDashboard.putNumber("MatchTime",DriverStation.getMatchTime());
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.drivetrain.checkcase=0;
        
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.timer.reset();
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
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        LimelightHelpers.SetIMUMode("limelight-l", 1);
    }

    @Override
    public void teleopPeriodic() {
     if (m_robotContainer.drivetrain.wonAuto){
            if (DriverStation.getMatchTime()==125 || DriverStation.getMatchNumber() == 75){
            testColor = new Color(255, 255, 0);
         }else {
            testColor = new Color(192, 192, 192);
         }
        }        
         if (!m_robotContainer.drivetrain.wonAuto){
            if (DriverStation.getMatchTime()==100 || DriverStation.getMatchNumber() == 50){
            testColor = new Color(255, 255, 0);
         }else {
            testColor = new Color(192, 192, 192);
         }
        }        
          SmartDashboard.putString("Alliance Shift", testColor.toString());
    }

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

    @Override
    public void simulationPeriodic() {}
}
