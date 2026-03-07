package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelShooter;
import frc.robot.subsystems.FuelShooterMax;
public class ShooterDecCommand extends Command{

    private FuelShooterMax fs;
    private double rpm;

    public ShooterDecCommand(FuelShooterMax fs,double rpm) {
        this.fs = fs;
        this.rpm = rpm;
        addRequirements(fs);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        
        

       fs.decreaseRPM();
       System.out.println("Flywheel RPM set to: " + rpm);
       SmartDashboard.putNumber("Shooter RPM Target", rpm);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
