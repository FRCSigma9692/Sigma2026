// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter4Sub;
import frc.robot.subsystems.TransferSub;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransferCmd extends Command {
  //private RobotContainer m_robotContainer = new RobotContainer();
  private TransferSub feedingsub;
  private double rpm;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  /** Creates a new FeedingCmd. */
  public TransferCmd(TransferSub feed, double speed, CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.feedingsub = feed;
    this.rpm = speed;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    addRequirements(feed);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feedingsub.runShooterRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
