// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ShotCalculator extends SubsystemBase {
//   public ShooterCalc shooterCalc;
//   public final double InitFlyWheelVel= 0;
//   public double FlyWheelVelOffset = InitFlyWheelVel;
//   CommandSwerveDrivetrain commandSwerveDrivetrain;
//   public final double LoopPeriod= 0.02;
//   public final double ShooterDistFromCenter = 0.3429;
  

//   /** Creates a new ShotCalculator. */
//   public ShotCalculator() {

//   }

//   @Override
//   public void periodic() {
//     Pose2d estimatPose2d = commandSwerveDrivetrain.GetPose();
//     ChassisSpeeds robotChassisSpeeds = commandSwerveDrivetrain.getState().Speeds;
//     estimatPose2d = 
//     estimatPose2d.exp(
//       new Twist2d(
//         robotChassisSpeeds.vxMetersPerSecond*LoopPeriod,
//         robotChassisSpeeds.vyMetersPerSecond*LoopPeriod,
//         robotChassisSpeeds.omegaRadiansPerSecond*LoopPeriod

//       )
//     );
//     Pose2d ShooterPose = new Pose2d(estimatPose2d.getX()+ShooterDistFromCenter, estimatPose2d.getY(), estimatPose2d.getRotation());
//     double ShooterToHub = commandSwerveDrivetrain.GetDistFromHub();
//     double RobotHeading = estimatPose2d.getRotation().getRadians();
//     double ShooterVelX = robotChassisSpeeds.vxMetersPerSecond + robotChassisSpeeds.omegaRadiansPerSecond * (ShooterDistFromCenter*Math.cos(RobotHeading));
//     double ShooterVelY = robotChassisSpeeds.vxMetersPerSecond + robotChassisSpeeds.omegaRadiansPerSecond * (ShooterDistFromCenter * Math.cos(RobotHeading));
//     double lookAheadDist = ShooterToHub;
//     for (int i = 0; i< 20; i++){
//     double OffsetX = ShooterVelX*shooterCalc.GetTOF(lookAheadDist);
//     double OffsetY = ShooterVelY * shooterCalc.GetTOF(lookAheadDist);
//     Translation2d lookaheadTranslation2d = ShooterPose.getTranslation().plus(new Translation2d(ShooterDistFromCenter,0));
//     double newDist = Math.sqrt(Math.pow((commandSwerveDrivetrain.HubY-lookaheadTranslation2d.getY()),2) + Math.pow((commandSwerveDrivetrain.HubX-lookaheadTranslation2d.getX()),2));
//     if (Math.abs(newDist-lookAheadDist)<0.01){
//       lookAheadDist = newDist;
//       break;
//     }
//     lookAheadDist = newDist;
//     }
//     double TOFFinal = shooterCalc.GetTOF(lookAheadDist);
//     Translation2d compenTranslation2d = ShooterPose.getTranslation().plus(new Translation2d(ShooterVelX*TOFFinal, ShooterVelY*TOFFinal));
//     double FlyWheelSpeed = 


      
    
//     // This method will be called once per scheduler run
//   }
  
// }
  
  

