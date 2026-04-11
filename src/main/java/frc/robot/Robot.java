// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.util.sendable.Sendable;
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
    double blinkcounter = 0;
   boolean blinkstatus = false;
   boolean blinkstatus2 = false;
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
    boolean LEDblink = false;
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


        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("Checkcase", checkcase);
    }
    

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        checkcase= true;
        LEDblink = false;

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
        alliance = DriverStation.getAlliance();
        Gamedata = DriverStation.getGameSpecificMessage();
        Time = DriverStation.getMatchTime();
        if (Gamedata.length()>0 && checkcase){
            if (Time < 140 && Time > 135 || Time < 30) {
                color = (new Color(255, 0, 0));
                // LED blink
            }
            //WonAuto if Red
            if (alliance.get()==Alliance.Red && Time>105 && checkcase) {
                checkcase = false;
                if (Gamedata.charAt(0)=='R')
                    wonAuto = true;
                 if (Gamedata.charAt(0)=='B')
                    wonAuto=false;
                
            }
            //WonAuto if Blue
            if (alliance.get() == Alliance.Blue && Time > 105 && checkcase) {
                checkcase = false;
                if (Gamedata.charAt(0)=='B')
                    wonAuto = true;
                 if (Gamedata.charAt(0) == 'R')
                    wonAuto = false;
                
            }
        }
        else {

        }
            
            if (!checkcase){
                if (
                 ((Time>105 && 110<Time) || (Time>80 && 85<Time)) || (Time>55 && Time<60) || (Time>30 && Time<35) || (Time>130 && Time<135)){
                   LEDblink = true;
                    //LED blink
                 }
                 if (!(((Time > 105 && 110 > Time) || (Time > 80 && 85 > Time)) || (Time > 55 && Time < 60)
                         || (Time > 30 && Time < 35) || (Time > 130 && Time < 135))) {
                     LEDblink = false;
                     // LED blink
                 }
                 
                 if (alliance.get() == Alliance.Red && Gamedata.charAt(0) == 'B' ) {
                     //color = (new Color(255, 0, 0));
                     // LED blink
                     blinkstatus2 = false;
                 }
                 if (alliance.get() == Alliance.Red && Gamedata.charAt(0) == 'R') {
                     blinkstatus2 = true;
                     // LED blink
                 }
                 if (alliance.get() == Alliance.Blue && Gamedata.charAt(0) == 'R') {
                     blinkstatus2 = false;
                     // LED blink
                 }if (alliance.get() == Alliance.Blue && Gamedata.charAt(0) == 'B') {
                     blinkstatus2 = true;
                     // LED blink
                 }
                 

            }
            if (LEDblink==true){
                if (blinkcounter%10==0){
                    blinkstatus=!blinkstatus;
                }
                blinkcounter++;
                
            }
        // SmartDashboard.putBoolean("LEDBLINK status", blinkstatus);
      //  SmartDashboard.putString("Active", color.toString());
        // SmartDashboard.putNumber("MatchTime",Time);
       // SmartDashboard.putString("Alliance", alliance.toString());
        // SmartDashboard.putString("Gamedata", Gamedata);
        // SmartDashboard.putBoolean("WonAuto: ",wonAuto);
        SmartDashboard.putNumber("LimelightPipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight-l"));
       // SmartDashboard.putString("Alliance Shift", testColor.toString());
        
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
