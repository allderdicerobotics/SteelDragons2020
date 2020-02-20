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

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Tube extends SubsystemBase {
  private final CANSparkMax tubeMotor;
  private CANPIDController tubeMotorPIDController;
  private CANEncoder tubeMotorCANEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxAccel;
  private double currentTubePosition;

  private double[] limeLightValues;

  public Tube() {
    tubeMotor = new CANSparkMax(Constants.CAN_TUBE, MotorType.kBrushless);
    tubeMotorPIDController = tubeMotor.getPIDController();
    tubeMotorCANEncoder = tubeMotor.getEncoder();

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kIz = 0.0;
    kFF = 0.0;
    kMaxOutput = 0.0;
    kMinOutput = 0.0;

    maxAccel = 0.0;

    tubeMotorPIDController.setP(kP);
    tubeMotorPIDController.setI(kI);
    tubeMotorPIDController.setD(kD);
    tubeMotorPIDController.setIZone(kIz);
    tubeMotorPIDController.setFF(kFF);
    tubeMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

    tubeMotorPIDController.setSmartMotionMaxAccel(maxAccel, 0);

    // //Use this only for tuning PID values and testing
    // SmartDashboard.putNumber("Tube P", kP);
    // SmartDashboard.putNumber("Tube I", kI);
    // SmartDashboard.putNumber("Tube D" , kD);
    // SmartDashboard.putNumber("Tube I Zone", kIz);
    // SmartDashboard.putNumber("Tube Feed Forward", kFF);
    // SmartDashboard.putNumber("Tube Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Tube Min Output", kMinOutput);

    // SmartDashboard.putNumber("Tube Max Accel", maxAccel);
    // SmartDashboard.putNumber("Tube Set Position", 0);
  }

  public void up() {
    setPosition(currentTubePosition + Constants.TUBE_MANUAL_CHANGE);
  }

  public void down() {
    setPosition(currentTubePosition - Constants.TUBE_MANUAL_CHANGE);
  }

  public void bottomPosition() {
    setPosition(0.0);
  }

  public void topPosition() {
    setPosition(300.0);
  }

  public void setAngle(double angle) {
  }

  public double getDistanceFromTarget() {
    limeLightValues = RobotContainer.getLimeLightValues();
    double currentYPosition = limeLightValues[2];
    return (78.0/Math.tan(Math.toRadians(32.5 + currentYPosition)));
  }

  public void setPosition(double position){
    tubeMotorPIDController.setReference(position, ControlType.kSmartMotion);
    currentTubePosition = position;
  }

  
  //DC Speed
  public void DCSetUp() {
    DCSetSpeed(Constants.TUBE_SPEED_UP);
  }

  public void DCSetDown() {
    DCSetSpeed(-Constants.TUBE_SPEED_DOWN);
  }  

  public void DCSetZero() {
    DCSetSpeed(0.0);
  }

  public void DCSetSpeed(double speed) {
    tubeMotorPIDController.setReference(speed, ControlType.kDutyCycle);
  }

  public void PIDSetup() {
  //   //Use this only for tuning PID values and testing
  //   double p = SmartDashboard.getNumber("Tube P", 0);
  //   double i = SmartDashboard.getNumber("Tube I", 0);
  //   double d = SmartDashboard.getNumber("Tube D", 0);
  //   double iz = SmartDashboard.getNumber("Tube I Zone", 0);
  //   double ff = SmartDashboard.getNumber("Tube Feed Forward", 0);
  //   double max = SmartDashboard.getNumber("Tube Max Output", 0);
  //   double min = SmartDashboard.getNumber("Tube Min Output", 0);

  //   double maxA = SmartDashboard.getNumber("Tube Max Accel", 0);

  //   if(p != kP) { kP = p; tubeMotorPIDController.setP(p); }
  //   if(i != kI) { kI = i; tubeMotorPIDController.setI(i); }
  //   if(d != kD) { kD = d; tubeMotorPIDController.setD(d); }
  //   if(iz != kIz) { kIz = iz; tubeMotorPIDController.setIZone(iz);  }
  //   if(ff != kFF) { kFF = ff; tubeMotorPIDController.setFF(ff);}
  //   if(max != kMaxOutput) || (min != kMinOutput) {
  //    tubeMotorPIDController.setOutputRange(min, max);
  //    kMaxOutput = max; kMinOutput = min; }

  //   if(maxA != maxAccel) { maxAccel = maxA; tubeMotorPIDController.setSmartMotionMaxAccel(maxA, 0);}

  //   double setPoint;
  //   setPoint = SmartDashboard.getNumber("Tube Set Position", 0);
  
  //   tubeMotorPIDController.setReference(setPoint, ControlType.kSmartMotion);
  
  //   SmartDashboard.putNumber("Current Tube Position: ", tubeMotorCANEncoder.getPosition());

  //   currentTubePosition = tubeMotorCANEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
