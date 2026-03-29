// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  public double pos;
  private final SparkMax hopper;
  SparkClosedLoopController closedLoopController;
  SparkMaxConfig configure = new SparkMaxConfig();
  RelativeEncoder hoppereEncoder;

  /** Creates a new Intake. */
  public Hopper() {
    hopper = new SparkMax(16, MotorType.kBrushless);
    hoppereEncoder = hopper.getEncoder();
    configure
        .inverted(false)
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    configure.encoder
        .velocityConversionFactor(1)
        .positionConversionFactor(1);
    configure.closedLoop
        .pid(0, 0, 0)
        .velocityFF(0)
        .outputRange(-0.8, 0.8);
    hopper.configure(configure, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    pos = hoppereEncoder.getPosition();
    SmartDashboard.putNumber("Hopper position", hoppereEncoder.getPosition());

    // This method will be called once per scheduler run
  }

  public void runHopper(double pow) {
    if (pow < 0 && pos > 0 || pow>0 && pos<44) {
      hopper.set(pow);
    } else {
      hopper.set(0);
    }
    SmartDashboard.updateValues();
  }
}
