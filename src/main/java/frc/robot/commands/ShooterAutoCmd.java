package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
public class ShooterAutoCmd extends Command{

    private ShooterSub shooter;


    public ShooterAutoCmd(ShooterSub shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
       shooter.runShooterRPM(2000);

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
