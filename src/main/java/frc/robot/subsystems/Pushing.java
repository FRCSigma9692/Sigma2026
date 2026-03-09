// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pushing extends SubsystemBase {
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
  public Pushing() {
    Pusher = new SparkMax(16, MotorType.kBrushless);
  //  PusherEncoder = Pusher.getEncoder();
  //   configure
  //   .smartCurrentLimit(20)
  //   .idleMode(IdleMode.kCoast);
  //   configure.encoder
  //   .velocityConversionFactor(1)
  //   .positionConversionFactor(1);
  //   configure.closedLoop
  //   .pid(Kp, Ki, Kd)
  //   .velocityFF(Kf)
  //   .minOutput(-1)
  //   .maxOutput(1);

    //Pusher.configure(configure, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // pushController = Pusher.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runPusher(){
    Pusher.set(-1);
  }
  public void Stop(){
    Pusher.set(0);
  }
}
