// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Hopper extends SubsystemBase {

  public double pos;
  public boolean holdenabled = false;
  public final double Kp = 5;
  public double output;
  public final SparkMax hopper;
  SparkClosedLoopController closedLoopController;
  SparkMaxConfig configure = new SparkMaxConfig();
  RelativeEncoder hoppereEncoder;
  // CommandXboxController joystick;
  private CANcoder cancoder = new CANcoder(26, "Sigma9692");
  CommandXboxController joystick;

  /** Creates a new Intake. */
  public Hopper(CommandXboxController joystick) {

    this.joystick = joystick;
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
    pos = cancoder.getAbsolutePosition().getValueAsDouble();
    SmartDashboard.putNumber("Hopper position", pos);

    if ((pos < -0.27 && pos > -0.32) && (joystick.getRightY() < 0.15 && joystick.getRightY() > -0.15)) {
      hopper.set(0.3);
    } else if ((pos < -0.32 || pos > -0.27) && (joystick.getRightY() < 0.15 && joystick.getRightY() > -0.15)) {
      hopper.set(0);
    }
    
    // System.out.println("Hold enable running");
    SmartDashboard.putNumber("Ouput", hopper.getAppliedOutput());

    // if(pos < -0.265 && pos > -0.275 && joystick.getRightY()<0.1 &&
    // joystick.getRightY() >-0.1){
    // hopper.set(0.25);
    // }
    // else if(pos >= -0.285 && joystick.getRightY()<0.1 && joystick.getRightY()
    // >-0.1){
    // hopper.set(0);
    // }
    // This method will be called once per scheduler run
  }

  public void autoHopper(double pow) {
    if (hopper.getForwardLimitSwitch().isPressed()) {
      hopper.set(0);
    } else {
      hopper.set(pow);
    }
    SmartDashboard.updateValues();
  }

  public void runHopper(double pow) {
    if (pow < 0 && pos <= -0.01 || pow > 0 && pos > -0.308) {
      hopper.set(pow);
    } else {
      hopper.set(0);
    }
    SmartDashboard.updateValues();
  }

  public void stop() {

    hopper.set(0);

  }

  public void stopandHold() {
    holdenabled = true;
  }

  public void armRelease() {
    holdenabled = false;
  }
}