// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class FeedingSub extends SubsystemBase {

  // Motors

        private final SparkMax leftmotor;
       // private final SparkMax rightmotor;
        //private final RelativeEncoder shooterEncoder2;
        // Closed-loop objects (ONLY from leader)
        private final SparkClosedLoopController shooterController;
        private final RelativeEncoder shooterEncoder;
        public static double Kp = 0.0073; // 0.00055
        public static double Ki = 0.00000009;//0
        public static double Kd = 0.61; // 0.03
        public static double Kf = 0.000195;
    
    
        public FeedingSub() {
          
        leftmotor = new SparkMax(17, MotorType.kBrushless);
        //rightmotor = new SparkMax(18, MotorType.kBrushless);
    
        /* ---------------- Config Objects ---------------- */
        SparkMaxConfig leftconfig = new SparkMaxConfig();
        //SparkMaxConfig rightConfig = new SparkMaxConfig();
      
    
        //shooterEncoder2 = rightmotor.getEncoder();
        shooterEncoder = leftmotor.getEncoder();
          leftconfig
          .smartCurrentLimit(30)
          .idleMode(IdleMode.kCoast);
          leftconfig.encoder
          .velocityConversionFactor(1)
          .positionConversionFactor(1);
          leftconfig.closedLoop
          .pid(Kp, Ki, Kd)
          .velocityFF(Kf)
          .minOutput(-0.8)
          .maxOutput(0.8);
    
        // rightConfig
        //     .apply(leftconfig)
        //     .follow(leftmotor, false);
    
        leftmotor.configure(leftconfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //rightmotor.configure(rightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        shooterController = leftmotor.getClosedLoopController();
      }
    
      @Override
      public void periodic() {
        
          }
    
        //shooterController.setReference(InitRPM, ControlType.kVelocity);
        
        // This method will be called once per scheduler run
     
      public void runShooterRPM(double rpm) {
          leftmotor.set(rpm);
          //shooterController.setSetpoint(rpm, ControlType.kVelocity);
  }

  /** Stop shooter */
  public void stopShooter() {
      shooterController.setSetpoint(0, ControlType.kVoltage);
  }

  /** Get current shooter velocity */
  public double getShooterRPM() {
      return shooterEncoder.getVelocity();   
  }
}



 
