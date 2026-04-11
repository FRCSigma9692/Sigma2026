// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax Lintake;
  private final SparkMax Rintake;

  SparkMaxConfig Lconfig = new SparkMaxConfig();

  /** Creates a new Intake. */
  public Intake() {
    Lintake = new SparkMax(14, MotorType.kBrushless);
    Rintake = new SparkMax(15, MotorType.kBrushless);
    SparkMaxConfig RConfig = new SparkMaxConfig();
    Lconfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kCoast);
    RConfig
        .smartCurrentLimit(80)
        .apply(Lconfig)
        .follow(Lintake, true);
    Lintake.configure(Lconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Rintake.configure(RConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Left IntakeCurrent", GetCurrentL());
    SmartDashboard.putNumber("Right IntakeCurrent", GetCurrentR());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left IntakeCurrent", GetCurrentL());
    SmartDashboard.putNumber("Right IntakeCurrent", GetCurrentR());
    // This method will be called once per scheduler run

  }

  public void runIntake(double speed) {
    Lintake.set(speed);
  }

  public void StopIntake() {
    Lintake.set(0);
  }

  public void runIntakeForHop(double speed) {
    // if (speed>0){
    Lintake.set(speed);
    // }
  }

  public double GetCurrentL() {
    return Lintake.getOutputCurrent();
  }

  public double GetCurrentR() {
    return Rintake.getOutputCurrent();
  }
}
