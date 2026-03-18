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

        private final SparkMax TransferL;
        private final SparkMax TransferR;
        SparkMaxConfig leftconfig = new SparkMaxConfig();
        private final SparkClosedLoopController TransferController;
        private final RelativeEncoder TransferEncoder;
        public static double Kp = 0.0073; // 0.00055
        public static double Ki = 0.00000009;//0
        public static double Kd = 0.61; // 0.03
        public static double Kf = 0.000195;  
    
        public TransferSub() {
          
        TransferL = new SparkMax(17, MotorType.kBrushless);
        TransferR = new SparkMax(18,MotorType.kBrushless);
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        //rightTransfer = new SparkMax(18, TransferType.kBrushless);
    
        /* ---------------- Config Objects ---------------- */
        //SparkMaxConfig rightConfig = new SparkMaxConfig();
      
    
        //shooterEncoder2 = rightTransfer.getEncoder();
        TransferEncoder = TransferL.getEncoder();
          leftconfig
          .smartCurrentLimit(60)
          .idleMode(IdleMode.kCoast);
          leftconfig.closedLoop
          .pid(Kp, Ki, Kd)
          .velocityFF(Kf)
          .minOutput(-0.8)
          .maxOutput(0.8);
          leftconfig.closedLoop.maxMotion
          .maxAcceleration(12000)
          .allowedProfileError(50);
          leftconfig.encoder
          .velocityConversionFactor(1)
          .positionConversionFactor(1);
    
        rightConfig
            .follow(TransferR, true)
            .apply(leftconfig);
    
        TransferL.configure(leftconfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        TransferR.configure(rightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        TransferController = TransferL.getClosedLoopController();
      }
    
      @Override
      public void periodic() {
        SmartDashboard.putNumber("Left Transfer Current", TransferL.getOutputCurrent());
        SmartDashboard.putNumber("Right Transfer Current", TransferR.getOutputCurrent());
      }
     
  public void runShooterRPM(double rpm) {
    TransferController.setReference(rpm, ControlType.kVelocity);
  }
  /** Stop shooter */
  public void stopShooter() {
      TransferController.setSetpoint(0, ControlType.kVoltage);
  }

  /** Get current shooter velocity */
  public double getShooterRPM() {
      return TransferEncoder.getVelocity();   
  }
}



 
