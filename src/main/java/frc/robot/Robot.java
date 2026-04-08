// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot {
    String Gamedata;
    public Optional<Alliance> alliance;
    boolean Allianceshift;
    double Matchtime;
    public Color color;
    public double Time;
    boolean AutoResult = false;
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    public boolean wonAuto;
    public boolean checkcase = true;
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

        m_robotContainer.field.setRobotPose(m_robotContainer.drivetrain.GetPose());
        Matchtime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Active Zone or No ", Allianceshift);
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.drivetrain.checkcase = 0;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        LimelightHelpers.setPipelineIndex("limelight-l", 1);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            Timer.delay(0.2);
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("LimelightPipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight-l"));
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        LimelightHelpers.setPipelineIndex("limelight-l", 1);
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        Gamedata = DriverStation.getGameSpecificMessage();
        Time = DriverStation.getMatchTime();
        if (Gamedata.length()>0){
            //WonAuto if Red
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red && Time>105 && checkcase) {

                if (Gamedata.charAt(0)=='R')
                    wonAuto = true;
                else if (Gamedata.charAt(0)=='B')
                    wonAuto=false;
                checkcase = false;
            }
            //WonAuto if Blue
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue && Time > 105 && checkcase) {
                if (Gamedata.charAt(0) == 'B')
                    wonAuto = true;
                else if (Gamedata.charAt(0) == 'R')
                    wonAuto = false;
                checkcase = false;
            }

            if (Gamedata.charAt(0)=='R' && DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red){
                color = new Color(0, 255, 0);
            }
            
            if (!checkcase){
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red && Gamedata.charAt(0)=='B' &&
                 ((Time>80 && 85<Time) || (Time>30 && 35<Time))){
                    //LED blink
                 }
                 if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue && Gamedata.charAt(0) == 'R' &&
                         ((Time > 80 && 85 < Time) || (Time > 30 && 35 < Time))) {
                        //LED blink
                 }

          
            }

        }
        color = new Color(0, 0, 0);
        SmartDashboard.putNumber("LimelightPipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight-l"));
        SmartDashboard.putString("Alliance Shift", testColor.toString());
        
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
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
