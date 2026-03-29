// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
  private Shooter s4;
  private final SparkMax TransferL;
  SparkMaxConfig leftconfig = new SparkMaxConfig();
  private final SparkClosedLoopController TransferController;
  private final RelativeEncoder TransferEncoder;
  public static double Kp = 0.0073; // 0.00055
  public static double Ki = 0.00000009;// 0
  public static double Kd = 0.61; // 0.03
  public static double Kf = 0.000195;
  CommandSwerveDrivetrain commandSwerveDrivetrain;

  public TransferSub(Shooter s4, CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.s4 = s4;
    TransferL = new SparkMax(17, MotorType.kBrushless);

    TransferEncoder = TransferL.getEncoder();
    leftconfig
        .smartCurrentLimit(70)
        .idleMode(IdleMode.kCoast);
    leftconfig.closedLoop
        .pid(Kp, Ki, Kd)
        .velocityFF(Kf)
        .minOutput(-0.8)
        .maxOutput(0.8);
    leftconfig.closedLoop.maxMotion
        .maxAcceleration(10000)
        .allowedProfileError(50);
    leftconfig.encoder
        .velocityConversionFactor(1)
        .positionConversionFactor(1);

    TransferL.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TransferController = TransferL.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Transfer Current", TransferL.getOutputCurrent());

  }

  public void runShooterRPM(double rpm) {
    // if (s4.getShooterRPM() >= (s4.speed - 56)) {
    TransferL.set(rpm);
    // } else {
    // TransferL.set(0);
    // }
    // TransferL.set(rpm);
  }

  /** Stop shooter */
  public void stopShooter() {
    TransferL.set(0);
  }

  /** Get current shooter velocity */
  public double getShooterRPM() {
    return TransferEncoder.getVelocity();
  }
}