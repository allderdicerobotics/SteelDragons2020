/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  // private final CANSparkMax shooterMotorLeft;
  // private final CANSparkMax shooterMotorRight;
  // private CANPIDController shooterMotorPIDController;
  // private CANEncoder shooterMotorCANEncoder;
  // private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxAccel;

   public Shooter() {
  //   shooterMotorLeft = new CANSparkMax(Constants.CAN_SHOOTER_LEFT, MotorType.kBrushless);
  //   shooterMotorRight = new CANSparkMax(Constants.CAN_SHOOTER_RIGHT, MotorType.kBrushless);

  //   shooterMotorRight.follow(shooterMotorLeft, true);

  //   shooterMotorPIDController = shooterMotorLeft.getPIDController();
  //   shooterMotorCANEncoder = shooterMotorLeft.getEncoder();

  //   kP = 0.0;
  //   kI = 0.0;
  //   kD = 0.0;
  //   kIz = 0.0;
  //   kFF = 0.0;
  //   kMaxOutput = 0.0;
  //   kMinOutput = 0.0;

  //   maxAccel = 0.0;

  //   shooterMotorPIDController.setP(kP);
  //   shooterMotorPIDController.setI(kI);
  //   shooterMotorPIDController.setD(kD);
  //   shooterMotorPIDController.setIZone(kIz);
  //   shooterMotorPIDController.setFF(kFF);
  //   shooterMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

  //   shooterMotorPIDController.setSmartMotionMaxAccel(maxAccel, 0);

  //   // //Use this only for tuning PID values and testing
  //   // SmartDashboard.putNumber("Shooter P", kP);
  //   // SmartDashboard.putNumber("Shooter I", kI);
  //   // SmartDashboard.putNumber("Shooter D" , kD);
  //   // SmartDashboard.putNumber("Shooter I Zone", kIz);
  //   // SmartDashboard.putNumber("Shooter Feed Forward", kFF);
  //   // SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
  //   // SmartDashboard.putNumber("Shooter Min Output", kMinOutput);

  //   // SmartDashboard.putNumber("Shooter Max Accel", maxAccel);
  //   // SmartDashboard.putNumber("Shooter Set Velocity", 0);
    }

  // public void stop() {
  //   setSpeed(0.0);
  // }

  // public void maxSpeed() {
  //   setSpeed(1000);
  // }

  // public void setSpeed(double speed){
  //   shooterMotorPIDController.setReference(speed, ControlType.kSmartVelocity);
  // }

  // public void PIDSetup() {
  // //   //Use this only for tuning PID values and testing
  // //   double p = SmartDashboard.getNumber("Shooter P", 0);
  // //   double i = SmartDashboard.getNumber("Shooter I", 0);
  // //   double d = SmartDashboard.getNumber("Shooter D", 0);
  // //   double iz = SmartDashboard.getNumber("Shooter I Zone", 0);
  // //   double ff = SmartDashboard.getNumber("Shooter Feed Forward", 0);
  // //   double max = SmartDashboard.getNumber("Shooter Max Output", 0);
  // //   double min = SmartDashboard.getNumber("Shooter Min Output", 0);

  // //   double maxA = SmartDashboard.getNumber("Shooter Max Accel", 0);

  // //   if(p != kP) { kP = p; shooterMotorPIDController.setP(p); }
  // //   if(i != kI) { kI = i; shooterMotorPIDController.setI(i); }
  // //   if(d != kD) { kD = d; shooterMotorPIDController.setD(d); }
  // //   if(iz != kIz) { kIz = iz; shooterMotorPIDController.setIZone(iz);  }
  // //   if(ff != kFF) { kFF = ff; shooterMotorPIDController.setFF(ff);}
  // //   if(max != kMaxOutput) || (min != kMinOutput) {
  // //    shooterMotorPIDController.setOutputRange(min, max);
  // //    kMaxOutput = max; kMinOutput = min; }

  // //   if(maxA != maxAccel) { maxAccel = maxA; shooterMotorPIDController.setSmartMotionMaxAccel(maxA, 0);}

  // //   double setPoint;
  // //   setPoint = SmartDashboard.getNumber("Shooter Set Velocity", 0);
  
  // //   shooterMotorPIDController.setReference(setPoint, ControlType.kSmartVelocity);
  
  // //   SmartDashboard.putNumber("Current Shooter Velocity: ", shooterMotorCANEncoder.getVelocity());
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

