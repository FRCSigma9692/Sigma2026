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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class TransferSub extends SubsystemBase {

        private final SparkMax Transfer;
        private final SparkClosedLoopController shooterController;
        private final RelativeEncoder shooterEncoder;
        public static double Kp = 0.0073; // 0.00055
        public static double Ki = 0.00000009;//0
        public static double Kd = 0.61; // 0.03
        public static double Kf = 0.000195;  
    
        public TransferSub() {
          
        Transfer = new SparkMax(16, MotorType.kBrushless);
        //rightTransfer = new SparkMax(18, TransferType.kBrushless);
    
        /* ---------------- Config Objects ---------------- */
        SparkMaxConfig leftconfig = new SparkMaxConfig();
        //SparkMaxConfig rightConfig = new SparkMaxConfig();
      
    
        //shooterEncoder2 = rightTransfer.getEncoder();
        shooterEncoder = Transfer.getEncoder();
          leftconfig
          .smartCurrentLimit(50)
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
        //     .follow(Transfer, false);
    
        Transfer.configure(leftconfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //rightTransfer.configure(rightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        shooterController = Transfer.getClosedLoopController();
      }
    
      @Override
      public void periodic() {
        SmartDashboard.putNumber("FeedingCurrent", Transfer.getOutputCurrent());
        
          }
    
        //shooterController.setReference(InitRPM, ControlType.kVelocity);
        
        // This method will be called once per scheduler run
     
  public void runShooterRPM(double rpm) {
      Transfer.set(rpm);
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



 
