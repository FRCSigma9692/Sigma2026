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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class FuelShooterMax extends SubsystemBase {
  public double InitRPM = 2400;

  // Motors
    public double ElapsedTime;
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;
    private final RelativeEncoder shooterEncoder2;
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    // Closed-loop objects (ONLY from leader)
    private final SparkClosedLoopController shooterController;
    private final RelativeEncoder shooterEncoder;

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
    public static double Kp = 0.0073; // 0.00055
    public static double Ki = 0.00000009;//0
    public static double Kd = 0.61; // 0.03
    public static double Kf = 0.000195;
    Timer timer = new Timer(); //0.0001486
    public double RampDOwnRPM;
    // Shooter RPM
    public static final double SHOOTER_RPM = 3000;

    public FuelShooterMax() {
      
    leftMotor = new SparkFlex(19, MotorType.kBrushless);
    rightMotor = new SparkFlex(20, MotorType.kBrushless);

    /* ---------------- Config Objects ---------------- */
    
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    shooterEncoder2 = rightMotor.getEncoder();
    shooterEncoder = leftMotor.getEncoder();

    /* ---------------- Common Motor Settings ---------------- */
    leftConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kCoast);
  
    /* ---------------- Closed Loop (PID + FF) ---------------- */
    leftConfig.closedLoop
        .p(Kp)
        .i(Ki)
        .d(Kd)
        .velocityFF(Kf)
        // .velocityFF(0.00006340000254567713)
        .outputRange(-1, 1);
    leftConfig.closedLoop.maxMotion
    .cruiseVelocity(3000)
    .allowedProfileError(5)
    .maxAcceleration(4000)
    ;
      

    leftConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
          
  
    rightConfig
        .apply(leftConfig)
        .follow(leftMotor, true);

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
      shooterController.setReference(InitRPM, ControlType.kVelocity);
  }

  /** Stop shooter */
  public void stopShooter() {
      leftMotor.stopMotor();  
  }

  /** Get current shooter velocity */
  public double getShooterRPM() {
      return shooterEncoder.getVelocity();   
  }  

  public void RampDown(double rpm){
    
    // RampDOwnRPM  = rpm;
    // if (RampDOwnRPM<=0){
    //   RampDOwnRPM = RampDOwnRPM;
    // }
    //  else {
    //   RampDOwnRPM = RampDOwnRPM-5;
    //  }
    shooterController.setReference(RampDOwnRPM,ControlType.kVoltage);
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
