// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShotCalculator extends SubsystemBase {
   double FlyWheelSpeed;
  public ShooterCalc shooterCalc = new ShooterCalc();
  public final double InitFlyWheelVel= 0;
  public double FlyWheelVelOffset = InitFlyWheelVel;
  CommandSwerveDrivetrain commandSwerveDrivetrain;
  public final double LoopPeriod= 0.02;
  public final double ShooterDistFromCenter = 0.3429;
  

  /** Creates a new ShotCalculator. */
  public ShotCalculator(CommandSwerveDrivetrain commandSwerveDrivetrain) {
      this.commandSwerveDrivetrain = commandSwerveDrivetrain;
  }

  @Override
  public void periodic() {
    Pose2d estimatPose2d = commandSwerveDrivetrain.GetPose();
    ChassisSpeeds robotChassisSpeeds = commandSwerveDrivetrain.getState().Speeds;
    estimatPose2d = 
    estimatPose2d.exp(
      new Twist2d(
        robotChassisSpeeds.vxMetersPerSecond*LoopPeriod,
        robotChassisSpeeds.vyMetersPerSecond*LoopPeriod,
        robotChassisSpeeds.omegaRadiansPerSecond*LoopPeriod

      )
    );
    Translation2d shooterOffset =
    new Translation2d(ShooterDistFromCenter, 0)
        .rotateBy(estimatPose2d.getRotation());

Pose2d ShooterPose =
    new Pose2d(
        estimatPose2d.getTranslation().plus(shooterOffset),
        estimatPose2d.getRotation()
    );
    double ShooterToHub = commandSwerveDrivetrain.GetDistFromHub();
    double RobotHeading = estimatPose2d.getRotation().getRadians();
    double ShooterVelX = robotChassisSpeeds.vxMetersPerSecond + robotChassisSpeeds.omegaRadiansPerSecond * (ShooterDistFromCenter*Math.cos(RobotHeading));
    double ShooterVelY = robotChassisSpeeds.vyMetersPerSecond + robotChassisSpeeds.omegaRadiansPerSecond * (ShooterDistFromCenter * Math.sin(RobotHeading));
    double lookAheadDist = ShooterToHub;
    for (int i = 0; i< 20; i++){
    double OffsetX = ShooterVelX*shooterCalc.GetTOF(lookAheadDist);
    double OffsetY = ShooterVelY * shooterCalc.GetTOF(lookAheadDist);
    Translation2d lookaheadTranslation2d = ShooterPose.getTranslation().plus(new Translation2d(OffsetX, OffsetY));
    double newDist = Math.sqrt(Math.pow((commandSwerveDrivetrain.HubY-lookaheadTranslation2d.getY()),2) + Math.pow((commandSwerveDrivetrain.HubX-lookaheadTranslation2d.getX()),2));
    if (Math.abs(newDist-lookAheadDist)<0.01){
      lookAheadDist = newDist;
      break;
    }
    lookAheadDist = newDist;
    }
    double TOFFinal = shooterCalc.GetTOF(lookAheadDist);
    Translation2d compenTranslation2d = ShooterPose.getTranslation().plus(new Translation2d(ShooterVelX*TOFFinal, ShooterVelY*TOFFinal));
    FlyWheelSpeed = shooterCalc.getVelocity(compenTranslation2d.getDistance(new Translation2d(commandSwerveDrivetrain.HubX, commandSwerveDrivetrain.HubY)));
    
    


      
    
    // This method will be called once per scheduler run
  }
  public double GetFlywheelSpeed(){
    return FlyWheelSpeed;
  }
  
}
  
  

