// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot {
    public Optional Alliance;
    boolean Allianceshift;
    double Matchtime;
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

        m_robotContainer.field.setRobotPose(m_robotContainer.drivetrain.GetPose());
        Matchtime = DriverStation.getMatchTime();
        // SmartDashboard.putBoolean("Result Of
        // Selection",m_robotContainer.drivetrain.wonAuto);
        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Active Zone or No ", Allianceshift);
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

        // m_robotContainer.drivetrain.runOnce(m_robotContainer.drivetrain::seedFieldCentric);
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.drivetrain.checkcase = 0;
        // CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        LimelightHelpers.setPipelineIndex("limelight-l", 1);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        LimelightHelpers.setPipelineIndex("limelight-l", 1);
        SmartDashboard.putNumber("LimelightPipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight-l"));
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        LimelightHelpers.setPipelineIndex("limelight-l", 1);

        // m_robotContainer.shooter.NoPID(0);
        // m_robotContainer.transfer.runShooterRPM(0);
        // m_robotContainer.feeder.FeederNoPID(0);

        CommandScheduler.getInstance().cancelAll();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        // LimelightHelpers.SetIMUMode("limelight-l", 1);
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("LimelightPipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight-l"));
        // if (m_robotContainer.drivetrain.wonAuto) {
        // if (Matchtime >= 130 && (Matchtime <= 130.00 && Matchtime >= 105) ||
        // (Matchtime <= 80 && Matchtime >= 55)) {
        // Allianceshift = true;
        // testColor = new Color(0, 255, 0);
        // } else {
        // Allianceshift = false;
        // testColor = new Color(255, 0, 0);
        // }
        // }

        // if (!m_robotContainer.drivetrain.wonAuto) {
        // if (Matchtime > 130 && (Matchtime <= 105.00 && Matchtime >= 80) || (Matchtime
        // <= 55 && Matchtime >= 30)) {
        // Allianceshift = true;
        // testColor = new Color(0, 255, 0);
        // } else {
        // Allianceshift = false;
        // testColor = new Color(255, 0, 0);
        // }
        // }
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
