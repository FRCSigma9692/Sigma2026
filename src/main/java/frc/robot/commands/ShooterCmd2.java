package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCmd2 extends Command {

    private Shooter shooter;
    private double rpm;

    public ShooterCmd2(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        shooter.Stop();
    }

    @Override
    public void execute() {
        shooter.runShooterRPMFixed7(rpm);
        SmartDashboard.putNumber("Shooter RPM Target", rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
