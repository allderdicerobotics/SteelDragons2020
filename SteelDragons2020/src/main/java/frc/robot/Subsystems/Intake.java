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


public class Intake extends SubsystemBase {
  private final CANSparkMax intakeFoldMotor;
  private CANPIDController intakeFoldMotorPIDController;
  private CANEncoder intakeFoldMotorCANEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxAccel;
  private double currentIntakeFoldPosition;

  private final CANSparkMax intakeSpinMotor;

  public Intake() {
    intakeFoldMotor = new CANSparkMax(Constants.CAN_FOLD_INTAKE, MotorType.kBrushless);
    intakeFoldMotorPIDController = intakeFoldMotor.getPIDController();
    intakeFoldMotorCANEncoder = intakeFoldMotor.getEncoder();

    intakeSpinMotor = new CANSparkMax(Constants.CAN_SPIN_INTAKE, MotorType.kBrushless);

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kIz = 0.0;
    kFF = 0.0;
    kMaxOutput = 0.0;
    kMinOutput = 0.0;

    maxAccel = 0.0;

    intakeFoldMotorPIDController.setP(kP);
    intakeFoldMotorPIDController.setI(kI);
    intakeFoldMotorPIDController.setD(kD);
    intakeFoldMotorPIDController.setIZone(kIz);
    intakeFoldMotorPIDController.setFF(kFF);
    intakeFoldMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

    intakeFoldMotorPIDController.setSmartMotionMaxAccel(maxAccel, 0);

    // //Use this only for tuning PID values and testing
    // SmartDashboard.putNumber("IntakeFold P", kP);
    // SmartDashboard.putNumber("IntakeFold I", kI);
    // SmartDashboard.putNumber("IntakeFold D" , kD);
    // SmartDashboard.putNumber("IntakeFold I Zone", kIz);
    // SmartDashboard.putNumber("IntakeFold Feed Forward", kFF);
    // SmartDashboard.putNumber("IntakeFold Max Output", kMaxOutput);
    // SmartDashboard.putNumber("IntakeFold Min Output", kMinOutput);

    // SmartDashboard.putNumber("IntakeFold Max Accel", maxAccel);
    // SmartDashboard.putNumber("IntakeFold Set Position", 0);
  }

  //INTAKE SPIN
  public void spinIn() {
    setSpinSpeed(Constants.INTAKE_SPEED_IN);
  }

  public void spinOut() {
    setSpinSpeed(Constants.INTAKE_SPEED_OUT);
  }

  public void spinStop() {
    setSpinSpeed(0.0);
  }

  public void setSpinSpeed(double speed) {
    intakeSpinMotor.set(speed);
  }

  //INTAKE POSITION
  public void bottom() {
    setPosition(0.0);
  }

  public void top() {
    setPosition(300.0);
  }

  public void setPosition(double position){
    intakeFoldMotorPIDController.setReference(position, ControlType.kSmartMotion);
    currentIntakeFoldPosition = position;
  }

  public void PIDSetup() {
  //   //Use this only for tuning PID values and testing
  //   double p = SmartDashboard.getNumber("IntakeFold P", 0);
  //   double i = SmartDashboard.getNumber("IntakeFold I", 0);
  //   double d = SmartDashboard.getNumber("IntakeFold D", 0);
  //   double iz = SmartDashboard.getNumber("IntakeFold I Zone", 0);
  //   double ff = SmartDashboard.getNumber("IntakeFold Feed Forward", 0);
  //   double max = SmartDashboard.getNumber("IntakeFold Max Output", 0);
  //   double min = SmartDashboard.getNumber("IntakeFold Min Output", 0);

  //   double maxA = SmartDashboard.getNumber("IntakeFold Max Accel", 0);

  //   if(p != kP) { kP = p; intakeFoldMotorPIDController.setP(p); }
  //   if(i != kI) { kI = i; intakeFoldMotorPIDController.setI(i); }
  //   if(d != kD) { kD = d; intakeFoldMotorPIDController.setD(d); }
  //   if(iz != kIz) { kIz = iz; intakeFoldMotorPIDController.setIZone(iz);  }
  //   if(ff != kFF) { kFF = ff; intakeFoldMotorPIDController.setFF(ff);}
  //   if(max != kMaxOutput) || (min != kMinOutput) {
  //    intakeFoldMotorPIDController.setOutputRange(min, max);
  //    kMaxOutput = max; kMinOutput = min; }

  //   if(maxA != maxAccel) { maxAccel = maxA; intakeFoldMotorPIDController.setSmartMotionMaxAccel(maxA, 0);}

  //   double setPoint;
  //   setPoint = SmartDashboard.getNumber("IntakeFold Set Position", 0);
  
  //   intakeFoldMotorPIDController.setReference(setPoint, ControlType.kSmartMotion);
  
  //   SmartDashboard.putNumber("Current IntakeFold Position: ", intakeFoldMotorCANEncoder.getPosition());

  //   currentIntakeFoldPosition = intakeFoldMotorCANEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

