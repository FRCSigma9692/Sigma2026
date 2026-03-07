// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CheckSparkFlex extends SubsystemBase {
  private final SparkFlex leftMotor;
  SparkFlexConfig leftConfig = new SparkFlexConfig();
  private final RelativeEncoder shooterEncoder;
  /** Creates a new CheckSparkFlex. */
  public CheckSparkFlex() {
    leftMotor = new SparkFlex(110, MotorType.kBrushless);

    shooterEncoder = leftMotor.getEncoder();

    leftConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast);
  
    /* ---------------- Closed Loop (PID + FF) ---------------- */

    leftConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);


    leftMotor.configure(leftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    //rightMotor.configure(rightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    //shooterController = leftMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("ENcoderFLEX", shooterEncoder.getPosition());
   // SmartDashboard.putData("RPMLeftMotor", shooterEncoder);
  }
 
  
  public void runMootor(){
      leftMotor.set(0.2);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
