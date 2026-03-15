// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.ShooterSub;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedCmd extends Command {
  private FeederSub feed;
  private RobotContainer m_RobotContainer;
  /** Creates a new FeedingCmd. */
  public FeedCmd(FeederSub feed, RobotContainer robotContainer) {
    this.feed = feed;
    this.m_RobotContainer = robotContainer;
  //  addRequirements(pusher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterSub.SHOOTER_RPM< m_RobotContainer.rpm)
    feed.runPusher();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //pusher.Stop();
      super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
