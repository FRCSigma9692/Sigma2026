// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter4Sub;
// public class ShooterCmd extends Command{

//     private Shooter4Sub shooter;
//     private double rpm;

//     public ShooterCmd(Shooter4Sub shooter,double rpm) {
//         this.shooter = shooter;
//         this.rpm = rpm;
//         addRequirements(shooter);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // TODO Auto-generated method stub
//         super.end(interrupted);
//     }

//     @Override
//     public void execute() {
        
//        shooter.runShooterRPM(rpm);
//        System.out.println("Flywheel RPM set to: " + rpm);
//        SmartDashboard.putNumber("Shooter RPM Target", rpm);
//     }

//     @Override
//     public boolean isFinished() {
//         // TODO Auto-generated method stub
//         return true;
//     }
// }
