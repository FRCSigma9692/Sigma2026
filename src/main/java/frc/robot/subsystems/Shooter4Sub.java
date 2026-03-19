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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Shooter4Sub extends SubsystemBase {
  public double InitRPM = 2400;


    private  SparkMax MML1;
    private  SparkMax FML2;
    private SparkMax FMR1;
    private  SparkMax FMR2;
    SparkMaxConfig MML1Config = new SparkMaxConfig();
    // Closed-loop objects (ONLY from leader)
    private SparkClosedLoopController shooterController;
    private RelativeEncoder shooterEncoder;

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
    public static double Kp = 0.0007; // 0.00055
    public static double Ki = 1e-7;//0
    public static double Kd = 0.011; // 0.03
    public static double Kf = 0.00004; //0.0001486
    public double RampDOwnRPM;
    // Shooter RPM
    public static final double SHOOTER_RPM = 3000;

    public Shooter4Sub() {
      
    MML1 = new SparkMax(21, MotorType.kBrushless);
    FML2 = new SparkMax(22, MotorType.kBrushless);
    FMR1 = new SparkMax(23, MotorType.kBrushless);
    FMR2 = new SparkMax(24, MotorType.kBrushless);

    /* ---------------- Config Objects ---------------- */
    SparkMaxConfig FML2Config = new SparkMaxConfig();
    SparkMaxConfig FMR1Config = new SparkMaxConfig();
    SparkMaxConfig FMR2Config = new SparkMaxConfig();
    shooterEncoder = MML1.getEncoder();

    /* ---------------- Common Motor Settings ---------------- */
    MML1Config
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast);
    /* ---------------- Closed Loop (PID + FF) ---------------- */
    MML1Config.closedLoop
        .p(Kp)
        .i(Ki)
        .d(Kd)
        .velocityFF(Kf)
        // .velocityFF(0.00006340000254567713)
        .outputRange(-1, 1);
    MML1Config.closedLoop.maxMotion
    //.cruiseVelocity(3000)
    .allowedProfileError(50)
    .maxAcceleration(20000);
      
    MML1Config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);
          
    FML2Config
      .follow(MML1, true)
      .apply(MML1Config);
     FMR1Config
      .follow(MML1, false)
      .apply(MML1Config);
    FMR2Config
      .follow(MML1, true)
      .apply(MML1Config);

    MML1.configure(MML1Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    FML2.configure(FML2Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    FMR1.configure(FMR1Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    FMR2.configure(FMR2Config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    shooterController = MML1.getClosedLoopController();
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
