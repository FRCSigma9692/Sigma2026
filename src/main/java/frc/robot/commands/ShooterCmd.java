package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command {

    private Shooter shooter;
    private double rpm;

    public ShooterCmd(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        System.out.println("Shooter Cmd Initilize called");
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        shooter.Stop();
        
    }

    @Override
    public void execute() {

        System.out.println("Shooter CMD Execute called");
        shooter.runShooterRPMFixed7(rpm);
        SmartDashboard.putNumber("Shooter RPM Target", rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
