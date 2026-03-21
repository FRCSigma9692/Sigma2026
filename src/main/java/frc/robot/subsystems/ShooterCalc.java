// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /** Add your docs here. */
// public class ShooterCalc extends SubsystemBase {
//     final double g = 9.81;
//     final double GOAL_HEIGHT = 2.05;
//     final double  MAX_HEIGHT = 3;
//     double bestError = Double.MAX_VALUE;
//     double bestAngle = 0;
//     double bestVelocity = 0;

//     double minVelocity = 3;
//     double maxVelocity = 26;

//     double minAngle = 15;
//     double maxAngle = 90;
//     double D;
 
//     @Override
//     public void periodic(){

//         // only scan velocity
//         for (double v = minVelocity; v <= maxVelocity; v += 0.05) {
 
//             double v2 = v * v;
 
//             double root = v2 * v2 - g * (g * D * D + 2 * GOAL_HEIGHT * v2);
 
//             if (root < 0)
//                 continue;
 
//             double sqrt = Math.sqrt(root);
 
//             // choose low arc solution
//             double theta = Math.atan((v2 - sqrt) / (g * D));
 
//             double angleDeg = Math.toDegrees(theta);
 
//             if (angleDeg < minAngle || angleDeg > maxAngle)
//                 continue;
 
//             double vy = v * Math.sin(theta);
//             double vx = v * Math.cos(theta);
 
//             // check max height
//             double hMax = (vy * vy) / (2 * g);
 
//             if (hMax > MAX_HEIGHT)
//                 continue;
 
//             double t = D / vx;
 
//             double y = vy * t - 0.5 * g * t * t;
 
//             double error = Math.abs(y - GOAL_HEIGHT);
 
//             if (error < bestError) {
//                 bestError = error;
//                 bestAngle = angleDeg;
//                 bestVelocity = v;
//             }
//         }
 
//         SmartDashboard.putNumber("ErrorForShoot",bestError);
//         SmartDashboard.putNumber("Best Angle",bestAngle);
//         SmartDashboard.putNumber("Best Vel",bestVelocity);
    
//     }
//     public double GetAngle(double dist){
//       D=dist;
//       return bestAngle;
//     }
//     public double getVelocity(double dist){
//       D=dist;
//       return bestVelocity;
//     }
//     public double GetTOF(double dist){
//       double TOF = 2*getVelocity(dist)*Math.sin(GetAngle(dist))/g;
//       return TOF;
//     }
// }
// // 