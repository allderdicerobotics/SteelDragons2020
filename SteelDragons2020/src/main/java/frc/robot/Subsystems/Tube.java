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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Tube extends SubsystemBase {
  private final CANSparkMax tubeMotor;
  private CANPIDController tubeMotorPIDController;
  private CANEncoder tubeMotorCANEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private double currentTubePosition;

  public Tube() {
    tubeMotor = new CANSparkMax(Constants.CAN_TUBE, MotorType.kBrushless);
    tubeMotorPIDController = tubeMotor.getPIDController();
    tubeMotorCANEncoder = tubeMotor.getEncoder();

    kP = 0.0001;
    kI = 0.0;
    kD = 0.0;
    kIz = 0.0;
    kFF = 0.0001;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    maxRPM = 5700;

    maxVel = 10000;
    maxAcc = 25000;

    tubeMotorPIDController.setP(kP);
    tubeMotorPIDController.setI(kI);
    tubeMotorPIDController.setD(kD);
    tubeMotorPIDController.setIZone(kIz);
    tubeMotorPIDController.setFF(kFF);
    tubeMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    tubeMotorPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    tubeMotorPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    tubeMotorPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    tubeMotorPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // //Use this only for tuning PID values and testing
    // SmartDashboard.putNumber("Tube P Gain", kP);
    // SmartDashboard.putNumber("Tube I Gain", kI);
    // SmartDashboard.putNumber("Tube D Gain" , kD);
    // SmartDashboard.putNumber("Tube I Zone", kIz);
    // SmartDashboard.putNumber("Tube Feed Forward", kFF);
    // SmartDashboard.putNumber("Tube Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Tube Min Output", kMinOutput);

    // SmartDashboard.putNumber("Tube Max Velocity", maxVel);
    // SmartDashboard.putNumber("Tube Min Velocity", minVel);
    // SmartDashboard.putNumber("Tube Max Acceleration", maxAcc);
    // SmartDashboard.putNumber("Tube Allowed Closed Loop Error", allowedErr);
    // SmartDashboard.putNumber("Tube Set Position", 0);
    // SmartDashboard.putNumber("Tube Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Tube Mode", true);
  }

  public void up() {
    setPosition(currentTubePosition + Constants.TUBE_MANUAL_CHANGE);
  }

  public void down() {
    setPosition(currentTubePosition - Constants.TUBE_MANUAL_CHANGE);
  }

  public void bottomPosition() {
    setPosition(163.0);
    System.out.println("bottom");
  }

  public void topPosition() {
    setPosition(0.0);
    System.out.println("top");
  }

  public void setPosition(double position){
    tubeMotorPIDController.setReference(position, ControlType.kSmartMotion);
    currentTubePosition = position;
  }

  
  // //DC Speed
  // public void DCSetUp() {
  //   DCSetSpeed(Constants.TUBE_SPEED_UP);
  // }

  // public void DCSetDown() {
  //   DCSetSpeed(-Constants.TUBE_SPEED_DOWN);
  // }  

  // public void DCSetZero() {
  //   DCSetSpeed(0.0);
  // }

  // public void DCSetSpeed(double speed) {
  //   tubeMotorPIDController.setReference(speed, ControlType.kDutyCycle);
  // }

  public void PIDSetup() {
    //read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("Tube P Gain", 0);
    // double i = SmartDashboard.getNumber("Tube I Gain", 0);
    // double d = SmartDashboard.getNumber("Tube D Gain", 0);
    // double iz = SmartDashboard.getNumber("Tube I Zone", 0);
    // double ff = SmartDashboard.getNumber("Tube Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Tube Max Output", 0);
    // double min = SmartDashboard.getNumber("Tube Min Output", 0);
    // double maxV = SmartDashboard.getNumber("Tube Max Velocity", 0);
    // double minV = SmartDashboard.getNumber("Tube Min Velocity", 0);
    // double maxA = SmartDashboard.getNumber("Tube Max Acceleration", 0);
    // double allE = SmartDashboard.getNumber("Tube Allowed Closed Loop Error", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { tubeMotorPIDController.setP(p); kP = p; }
    // if((i != kI)) { tubeMotorPIDController.setI(i); kI = i; }
    // if((d != kD)) { tubeMotorPIDController.setD(d); kD = d; }
    // if((iz != kIz)) { tubeMotorPIDController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { tubeMotorPIDController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   tubeMotorPIDController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    // }
    // if((maxV != maxVel)) { tubeMotorPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    // if((minV != minVel)) { tubeMotorPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    // if((maxA != maxAcc)) { tubeMotorPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    // if((allE != allowedErr)) { tubeMotorPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    // double setPoint, processVariable;
    // boolean mode = SmartDashboard.getBoolean("Tube Mode", false);
    // if(mode) {
    //   setPoint = SmartDashboard.getNumber("Tube Set Velocity", 0);
    //   tubeMotorPIDController.setReference(setPoint, ControlType.kVelocity);
    //   processVariable = tubeMotorCANEncoder.getVelocity();
    // } else {
    //   setPoint = SmartDashboard.getNumber("Tube Set Position", 0);
    //   /**
    //    * As with other PID modes, Smart Motion is set by calling the
    //    * setReference method on an existing pid object and setting
    //    * the control type to kSmartMotion
    //    */
    //   tubeMotorPIDController.setReference(setPoint, ControlType.kSmartMotion);
    //   processVariable = tubeMotorCANEncoder.getPosition();
    // }

    // SmartDashboard.putNumber("Tube SetPoint", setPoint);
    // SmartDashboard.putNumber("Tube Process Variable", processVariable);
    // SmartDashboard.putNumber("Tube Output", tubeMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // PIDSetup();
  }
}
