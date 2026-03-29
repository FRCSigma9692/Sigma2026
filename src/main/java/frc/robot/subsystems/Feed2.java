// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed2 extends SubsystemBase {
  public static double Kp = 0.0004;
  public static double Ki = 0.0000017;
  public static double Kd = 0.011;
  public static double Kf = 0.00008;
  private final SparkMax Feeder;
  private final RelativeEncoder FeederEncoder;
  private final SparkClosedLoopController FeederController;

  private Shooter s4;
  CommandSwerveDrivetrain commandSwerveDrivetrain;

  SparkMaxConfig lConfig = new SparkMaxConfig();

  /** Creates a new Intake. */
  public Feed2(Shooter s4, CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.s4 = s4;
    Feeder = new SparkMax(25, MotorType.kBrushless);
    // FeederR = new SparkMax(20, MotorType.kBrushless);
    // SparkMaxConfig rConfig = new SparkMaxConfig();
    FeederEncoder = Feeder.getEncoder();
    // PusherEncoder = Pusher.getEncoder();
    lConfig
        .inverted(true)
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake);

    lConfig.closedLoop
        .pid(Kp, Ki, Kd)
        .velocityFF(Kf)
        .minOutput(-1)
        .maxOutput(1);
    lConfig.closedLoop.maxMotion
        .allowedProfileError(50)
        .maxAcceleration(10000);
    lConfig.encoder
        .velocityConversionFactor(1.0)
        .positionConversionFactor(1.0);
    // rConfig
    // .smartCurrentLimit(60)
    // .follow(Feeder, true)
    // .apply(lConfig);

    Feeder.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // FeederR.configure(rConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    FeederController = Feeder.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Pusher Current", GetCurrentL());
    // SmartDashboard.putNumber("Right Pusher Current",GetCurrentR());
    // SmartDashboard.putNumber("AppliedOutputFeeder",FeederL.getAppliedOutput());
    // if(s4.GetPow()>=s4.speed-0.005){
    // FeederNoPID(0.6);
    // }
    // else{
    // FeederNoPID(0);
    // }

    // This method will be called once per scheduler run
  }

  public void runFeeder(double vel) {
    FeederController.setReference(vel, ControlType.kMAXMotionVelocityControl);
  }

  public void FeederNoPID(double pow) {
    // if (s4.getShooterRPM() >= (s4.speed-56))
    Feeder.set(pow);
    // else {
    // Feeder.set(0);
    // }
  }

  public void Stop() {
    FeederController.setReference(0, ControlType.kVoltage);
  }

  public double GetCurrentL() {
    return Feeder.getOutputCurrent();
  }

  // public double GetCurrentR(){
  // return Feeder.getOutputCurrent();
  // }
  public double FeederVel() {
    return FeederEncoder.getVelocity();
  }

}
