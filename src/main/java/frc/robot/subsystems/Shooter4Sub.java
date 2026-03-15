// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Timestamp;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Shooter4Sub extends SubsystemBase {
  public double InitRPM = 2400;

  // Motors
    public double ElapsedTime;
    private final SparkMax MM;
    private final SparkMax FM1;
    private final SparkMax FM2;
    private final SparkMax FM3;
    private final RelativeEncoder shooterEncoder2;
    SparkMaxConfig MMConfig = new SparkMaxConfig();
    // Closed-loop objects (ONLY from leader)
    private final SparkClosedLoopController shooterController;
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder shooterEncoder3;
    private final RelativeEncoder shooterEncoder4;

    // public static double Kp = 0.0002; // 0.00007
    // public static double Ki = 0.00000014; //0.00000047
    // public static double Kd = 0.007; // 0.010
    // public static double Kf = 0.00004; //00006340000254567713
    
    // public static double Kp = 0.00007; // 0.00007
    // public static double Ki = 0.00000047; //0.0000004
  
    // public static double Kd = 0.010; // 0.010
    // public static double Kf = 0; //00006340000254567713
    public boolean reach1 = false;
    long Starttime = 0;
    public double StartingRPM = 2000;
    public double Poutput;
    public double currenttime;
    public static double Kp = 0.00053; // 0.00055
    public static double Ki = 0.00000;//0
    public static double Kd = 0.0013; // 0.03
    public static double Kf = 0.000165;
    Timer timer = new Timer(); //0.0001486
    public double RampDOwnRPM;
    // Shooter RPM
    public static final double SHOOTER_RPM = 3000;

    public Shooter4Sub() {
      
    MM = new SparkMax(30, MotorType.kBrushless);
    FM1 = new SparkMax(31, MotorType.kBrushless);
    FM2 = new SparkMax(32, MotorType.kBrushless);
    FM3 = new SparkMax(33, MotorType.kBrushless);

    /* ---------------- Config Objects ---------------- */
    
    SparkMaxConfig fm1Config = new SparkMaxConfig();
    SparkMaxConfig fm2Config = new SparkMaxConfig();
    SparkMaxConfig fm3Config = new SparkMaxConfig();
    shooterEncoder = MM.getEncoder();
    shooterEncoder2 = FM1.getEncoder();
    shooterEncoder3 = FM2.getEncoder();
    shooterEncoder4 = FM3.getEncoder();


    /* ---------------- Common Motor Settings ---------------- */
    MMConfig
        .inverted(true)
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast);
    /* ---------------- Closed Loop (PID + FF) ---------------- */
    MMConfig.closedLoop
        .p(Kp)
        .i(Ki)
        .d(Kd)
        .velocityFF(Kf)
        // .velocityFF(0.00006340000254567713)
        .outputRange(-1, 1);
    leftConfig.closedLoop.maxMotion
    //.cruiseVelocity(3000)
    .allowedProfileError(50)
    .maxAcceleration(12000)
    ;
      
    leftConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
          
    rightConfig
      .follow(leftMotor, true)
      .apply(leftConfig);

    leftMotor.configure(leftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    shooterController = leftMotor.getClosedLoopController();


  }

  @Override
  public void periodic() {
    // double vel = getShooterRPM();
    // if (vel>=2480){
    //   reach1 = true;
    // }
    //   if (((StartingRPM-4)>=vel) && reach1 && vel<=3000 && vel>1000){
    //       //timer.start();
          
    //       Poutput = (10*((StartingRPM-vel)));
    //       // InitRPM+=Poutput;
    //   }
    //   else {
    //     //timer.reset();
    //     InitRPM = 2400;
    //     Poutput = 0;
    //   }
    //shooterController.setReference(InitRPM, ControlType.kMAXMotionVelocityControl);
    SmartDashboard.putNumber("OutputPower", Poutput);
    SmartDashboard.putNumber("Timer", currenttime);
    SmartDashboard.putNumber("Changed KP:",Kp);
    SmartDashboard.putNumber("Changed Ki:",Ki);
    SmartDashboard.putNumber("Changed Kd:",Kd);

    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putNumber("RPMLeftMotor", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("RPMRightMotor",shooterEncoder2.getVelocity());
    SmartDashboard.putNumber("InitRPM", InitRPM);
    SmartDashboard.putNumber("RPM", getShooterRPM());
    // This method will be called once per scheduler run
  }
  public void increaseRPM(){
      if (InitRPM <= 4000){
        InitRPM+=50;}
      else{
        InitRPM = 4000;
      }
  }

  public void decreaseRPM(){
    
    if (InitRPM<=0){
      InitRPM = 0;
    }
    else {
      InitRPM-=50;
    }
  }
  public void runShooterRPM(double rpm) {
   //   leftMotor.set(0.5);
      shooterController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
  }

  /** Get current shooter velocity */
  public double getShooterRPM() {
      return shooterEncoder.getVelocity();   
  }  

  public void Stop(){
    shooterController.setReference(0,ControlType.kVoltage);
  }

  public static double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
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
