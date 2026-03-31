// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CommandSwerveDrivetrain commandSwerveDrivetrain;
  public double InitRPM = 2000;
  private SparkMax M1;
  private SparkMax M2;
  SparkMaxConfig M1Config = new SparkMaxConfig();
  // Closed-loop objects (ONLY from leader)
  private SparkClosedLoopController shooterController;
  private RelativeEncoder shooterEncoder;
  public boolean reach1 = false;
  public double speed;
  long Starttime = 0;
  public double StartingRPM = 2000;
  public double Poutput;
  public double currenttime;
  public static double Kp = 0.0009;// 0.001; // 0.00055 //0.00072
  public static double Ki = 0;// 0;// 1e-7;//0 // 1e-9
  public static double Kd = 0;// 0.0007;// 0.011; // 0.0007
  public static double Kf = 0.00018;// 0.00025; // 0.00004; // 0.00006
  public double RampDOwnRPM;
  // Shooter RPM
  public static final double SHOOTER_RPM = 2600;

  public Shooter(CommandSwerveDrivetrain commandSwerveDrivetrain) {

    this.commandSwerveDrivetrain = commandSwerveDrivetrain;

    M1 = new SparkMax(22, MotorType.kBrushless);
    M2 = new SparkMax(23, MotorType.kBrushless);

    /* ---------------- Config Objects ---------------- */
    SparkMaxConfig M2Config = new SparkMaxConfig();
    shooterEncoder = M1.getEncoder();

    /* ---------------- Common Motor Settings ---------------- */
    M1Config
        .inverted(true)
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast);
    /* ---------------- Closed Loop (PID + FF) ---------------- */
    M1Config.closedLoop
        .p(Kp)
        .i(Ki)
        .d(Kd)
        .velocityFF(Kf)
        // .velocityFF(0.00006340000254567713)
        .outputRange(-1, 1);
    M1Config.closedLoop.maxMotion
        .cruiseVelocity(2600)
        .allowedProfileError(50)
        .maxAcceleration(20000);

    M1Config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    M2Config
        .follow(M1, true)
        .apply(M1Config);

    M1.configure(M1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    M2.configure(M2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterController = M1.getClosedLoopController();
  }

  @Override
  public void periodic() {
    speed = commandSwerveDrivetrain.shooterspeed;

    SmartDashboard.putNumber("OutputPower", Poutput);
    SmartDashboard.putNumber("Timer", currenttime);
    SmartDashboard.putNumber("Changed KP:", Kp);
    SmartDashboard.putNumber("Changed Ki:", Ki);
    SmartDashboard.putNumber("Changed Kd:", Kd);

    SmartDashboard.putNumber("RPMLeftMotor", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("RPM", getShooterRPM());
    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Applied Output", GetPow());
    // This method will be called once per scheduler run
  }

  public void increaseRPM() {

    if (InitRPM <= 4000) {
      InitRPM += 50;
    } else {
      InitRPM = 4000;
    }
  }

  public void decreaseRPM() {
    if (InitRPM <= 0) {
      InitRPM = 0;
    } else {
      InitRPM -= 50;
    }
  }

  public void runShooterRPM() {
    // shooterController.setReference(0.3, ControlType.kMAXMotionVelocityControl);
    M1.set(speed);
    SmartDashboard.putNumber("Set Speed", speed);
  }

  public void runShooterRPMFixed(double speed) {
    // shooterController.setReference(0.3, ControlType.kMAXMotionVelocityControl);
    M1.set(speed);
    SmartDashboard.putNumber("Set Speed", speed);
  }

  public void runShooterRPMFixed7(double speed) {
    shooterController.setReference(speed, ControlType.kMAXMotionVelocityControl);
    // M1.set(0.7);
    SmartDashboard.putNumber("Set Speed", speed);
  }

  /** Get current shooter velocity */
  public double getShooterRPM() {
    return shooterEncoder.getVelocity();
  }

  public void Stop() {
    shooterController.setReference(0, ControlType.kVoltage);
  }

  public void NoPID(double vel) {
    M1.set(vel);
  }

  public double GetVel() {
    return shooterEncoder.getVelocity();
  }

  public double GetPow() {
    return M1.getAppliedOutput();
  }

  public static double mapRange(double value, double inputStart, double inputEnd, double outputStart,
      double outputEnd) {
    // Avoid division by zero if the input range is zero
    if (inputStart == inputEnd) {
      throw new IllegalArgumentException("Input range cannot be zero.");
    }

    // Calculate the proportion of the value within the input range
    double proportion = (value - inputStart) / (inputEnd - inputStart);

    // Apply the proportion to the output range and shift by the output start
    return outputStart + proportion * (outputEnd - outputStart);
  }
}
