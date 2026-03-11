// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedingSub;
import frc.robot.subsystems.Pushing;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PushCmd extends Command {
  private Pushing pusher;
  /** Creates a new FeedingCmd. */
  public PushCmd(Pushing pushing) {
    this.pusher = pushing;
  //  addRequirements(pusher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pusher.runPusher();
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
