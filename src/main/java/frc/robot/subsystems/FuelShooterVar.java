package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooterVar extends SubsystemBase {

    private SparkFlex leftMotor;
    private SparkFlex rightMotor;

    private RelativeEncoder shooterEncoder;
    private RelativeEncoder shooterEncoder2;
    private SparkClosedLoopController shooterController;

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    public double Kp = 0.00007;
    public double Ki = 0.00000047;
    public double Kd = 0.010;
    public double Kf = 0;

    public double multiple = 1;
    public char changing = 'p';

    public static final double SHOOTER_RPM = 4000;

    public FuelShooterVar() {

        leftMotor = new SparkFlex(21, MotorType.kBrushless);
        rightMotor = new SparkFlex(22, MotorType.kBrushless);

        shooterEncoder = leftMotor.getEncoder();
        shooterEncoder2 = rightMotor.getEncoder();

        applyPIDValues();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Kp", Kp);
        SmartDashboard.putNumber("Shooter Ki", Ki);
        SmartDashboard.putNumber("Shooter Kd", Kd);
        SmartDashboard.putNumber("Shooter Kf", Kf);

        double newKp = SmartDashboard.getNumber("Kp", Kp);
        double newKi = SmartDashboard.getNumber("Ki", Ki);
        double newKd = SmartDashboard.getNumber("Kd", Kd);
        double newKf = SmartDashboard.getNumber("Kf", Kf);

        if (newKp != Kp || newKi != Ki || newKd != Kd || newKf != Kf) {
          Kp = newKp;
          Ki = newKi;
          Kd = newKd;
          Kf = newKf;

        applyPIDValues();
        }



        SmartDashboard.putNumber("Shooter Multiple", multiple);
        SmartDashboard.putString("Shooter Selected", String.valueOf(changing).toUpperCase());

        SmartDashboard.putNumber("Shooter RPM Left", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter RPM Right", shooterEncoder2.getVelocity());
    }

    public void runShooterRPM(double rpm) {
        shooterController.setReference(rpm, ControlType.kVelocity);
    }

    public void stopShooter() {
        leftMotor.stopMotor();
    }

    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    public void increaseMultiple() {
        multiple *= 10;
    }

    public void decreaseMultiple() {
        multiple /= 10;
        if (multiple < 1e-9) {
            multiple = 1e-9;
        }
    }

    public void nextVariable() {
        if (changing == 'p') changing = 'i';
        else if (changing == 'i') changing = 'd';
        else if (changing == 'd') changing = 'f';
        else changing = 'p';
    }

    public void previousVariable() {
        if (changing == 'p') changing = 'f';
        else if (changing == 'f') changing = 'd';
        else if (changing == 'd') changing = 'i';
        else changing = 'p';
    }

    public void increasePIDValue() {
        if (changing == 'p') Kp += multiple;
        else if (changing == 'i') Ki += multiple;
        else if (changing == 'd') Kd += multiple;
        else if (changing == 'f') Kf += multiple;

        applyPIDValues();
    }

    public void decreasePIDValue() {
        if (changing == 'p') Kp -= multiple;
        else if (changing == 'i') Ki -= multiple;
        else if (changing == 'd') Kd -= multiple;
        else if (changing == 'f') Kf -= multiple;

        if (Kp < 0) {
          Kp += multiple;
        } 
        if (Ki < 0) {
          Ki += multiple;
        }
        
        if (Kd < 0) {
          Kd += multiple;
        }
        
        if (Kf < 0) {
          Kf += multiple;
        }

        applyPIDValues();
    }

    public void applyPIDValues() {

        leftConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);

        leftConfig.closedLoop
            .p(Kp)
            .i(Ki)
            .d(Kd)
            .velocityFF(Kf)
            .outputRange(-1, 1);

        leftConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        rightConfig
            .apply(leftConfig)
            .follow(leftMotor, true);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterController = leftMotor.getClosedLoopController();
    }
}
