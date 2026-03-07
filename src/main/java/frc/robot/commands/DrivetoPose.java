// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivetoPose extends Command {
  CommandPS5Controller User1 = new CommandPS5Controller(0);
  PathConstraints constraints;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private final Pose2d targetPose;
  private Command pathCommand;
  /** Creates a new DrivetoPose. */
  public DrivetoPose(CommandSwerveDrivetrain commandSwerveDrivetrain, Pose2d targPose2d) {
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.targetPose = targPose2d;
    addRequirements(commandSwerveDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    constraints = new PathConstraints(1, 1, Math.toRadians(500), Math.toRadians(600));
    pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    if ((Math.abs(User1.getLeftX()))<0.1 && Math.abs(User1.getLeftY())<0.1 && Math.abs(User1.getRightX())<0.1 && Math.abs(User1.getRightY())<0.1){
        CommandScheduler.getInstance().schedule(pathCommand);
        SmartDashboard.putString("Scheduled", "true");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!((Math.abs(User1.getLeftX()))<0.04 && Math.abs(User1.getLeftY())<0.04 && Math.abs(User1.getRightX())<0.04 && Math.abs(User1.getRightY())<0.04)){
        if (pathCommand.isScheduled()){
          pathCommand.cancel();
        }
    }
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
