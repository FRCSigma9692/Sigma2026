// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSub extends SubsystemBase {
  public static double Kp = 0.1;
  public static double Ki = 0;
  public static double Kd = 0.001;
  public static double Kf = 0.000000001;
  private final SparkMax Pusher;
  // private final RelativeEncoder PusherEncoder;
  // private final SparkClosedLoopController pushController;
  //private final SparkClosedLoopController TransferController;
  //SparkMaxConfig configure = new SparkMaxConfig();
  /** Creates a new Intake. */
  public FeederSub() {
    Pusher = new SparkMax(17, MotorType.kBrushless);
    SparkMaxConfig configure = new SparkMaxConfig();
  //  PusherEncoder = Pusher.getEncoder();
    configure
    .smartCurrentLimit(30)
    .idleMode(IdleMode.kCoast);
    configure.encoder
    .velocityConversionFactor(1)
    .positionConversionFactor(1);
    configure.closedLoop
    .pid(Kp, Ki, Kd)
    .velocityFF(Kf)
    .minOutput(-1)
    .maxOutput(1);

    //Pusher.configure(configure, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // pushController = Pusher.getClosedLoopController();
  }

  @Override
  public void periodic() {
            SmartDashboard.putNumber("Pusher Current",GetCurrent());
    // This method will be called once per scheduler run
  }
  public void runPusher(){
    Pusher.set(1);
  }
  public void ReversePusher(){
    Pusher.set(-1);
  }
  public void Stop(){
    Pusher.set(0);
  }
  public double GetCurrent(){
    return Pusher.getOutputCurrent();
  }
 
}
