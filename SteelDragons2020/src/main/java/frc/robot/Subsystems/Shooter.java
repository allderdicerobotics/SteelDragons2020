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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotorLeft;
  private CANPIDController shooterMotorLeftPIDController;
  private CANEncoder shooterMotorLeftCANEncoder;

  private final CANSparkMax shooterMotorRight;
  private CANPIDController shooterMotorRightPIDController;
  private CANEncoder shooterMotorRightCANEncoder;

  private double kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput, maxAccel;

  double speed;

  public Shooter() {
    shooterMotorLeft = new CANSparkMax(Constants.CAN_SHOOTER_LEFT, MotorType.kBrushless);
    shooterMotorLeftPIDController = shooterMotorLeft.getPIDController();
    shooterMotorLeftCANEncoder = shooterMotorLeft.getEncoder();

    shooterMotorRight = new CANSparkMax(Constants.CAN_SHOOTER_RIGHT, MotorType.kBrushless);
    shooterMotorRightPIDController = shooterMotorRight.getPIDController();
    shooterMotorRightCANEncoder = shooterMotorRight.getEncoder();

    shooterMotorLeft.restoreFactoryDefaults();
    shooterMotorRight.restoreFactoryDefaults();

    shooterMotorLeft.setIdleMode(IdleMode.kCoast);
    shooterMotorRight.setIdleMode(IdleMode.kCoast);

    kP = 0.0002;
    kFF = 0.00023;

    kI = 0.0;
    kD = 0.0;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    kIz = 0.0;
    speed = 0.0;
    maxAccel = 0.0;

    shooterMotorRightPIDController.setP(kP);
    shooterMotorRightPIDController.setI(kI);
    shooterMotorRightPIDController.setD(kD);
    shooterMotorRightPIDController.setFF(kFF);

    shooterMotorLeftPIDController.setP(kP);
    shooterMotorLeftPIDController.setI(kI);
    shooterMotorLeftPIDController.setD(kD);
    shooterMotorLeftPIDController.setFF(kFF);


    // motorPIDSetup(shooterMotorRightPIDController);
    // motorPIDSetup(shooterMotorLeftPIDController);

    // SmartDashboard.putNumber("Speed", speed);


    // //Use this only for tuning PID values and testing
    // SmartDashboard.putNumber("Shooter P", kP);
    // SmartDashboard.putNumber("Shooter I", kI);
    // SmartDashboard.putNumber("Shooter D" , kD);
    // SmartDashboard.putNumber("Shooter I Zone", kIz);
    // SmartDashboard.putNumber("Shooter Feed Forward", kFF);
    // SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Shooter Min Output", kMinOutput);

    // SmartDashboard.putNumber("Shooter Max Accel", maxAccel);
    // SmartDashboard.putNumber("Shooter Set Velocity", 0);
  }

  public void motorPIDSetup(CANPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kFF);
  }

  public void stop() {
    setSpeed(0.0, 0.0);
  }

  public void maxSpeed() {
    setSpeed(Constants.SHOOTER_MAX_RPM, -Constants.SHOOTER_MAX_RPM);
  }

  public void normalSpeed() {
    setSpeed(Constants.SHOOTER_NORMAL_SPEED, -Constants.SHOOTER_NORMAL_SPEED);
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    shooterMotorLeftPIDController.setReference(leftSpeed, ControlType.kVelocity);
    shooterMotorRightPIDController.setReference(rightSpeed, ControlType.kVelocity);
  }

  public void setColorWheelFastSpeed() {
    setSpeed(Constants.COLOR_WHEEL_FAST_SPEED, Constants.COLOR_WHEEL_FAST_SPEED);
  }
  
  public void setColorWheelSlowSpeed() {
    setSpeed(-Constants.COLOR_WHEEL_SLOW_SPEED, -Constants.COLOR_WHEEL_SLOW_SPEED);
  }

  public void PIDSetup() {
    //Use this only for tuning PID values and testing
    double p = SmartDashboard.getNumber("Shooter P", 0);
    double i = SmartDashboard.getNumber("Shooter I", 0);
    double d = SmartDashboard.getNumber("Shooter D", 0);
    double iz = SmartDashboard.getNumber("Shooter I Zone", 0);
    double ff = SmartDashboard.getNumber("Shooter Feed Forward", 0);
    double max = SmartDashboard.getNumber("Shooter Max Output", 0);
    double min = SmartDashboard.getNumber("Shooter Min Output", 0);

    double maxA = SmartDashboard.getNumber("Shooter Max Accel", 0);

    if(p != kP) { kP = p; shooterMotorLeftPIDController.setP(p); shooterMotorRightPIDController.setP(p); }
    if(i != kI) { kI = i; shooterMotorLeftPIDController.setI(i); shooterMotorRightPIDController.setI(i); }
    if(d != kD) { kD = d; shooterMotorLeftPIDController.setD(d); shooterMotorRightPIDController.setD(d); }
    if(iz != kIz) { kIz = iz; shooterMotorLeftPIDController.setIZone(iz); shooterMotorRightPIDController.setIZone(iz); }
    if(ff != kFF) { kFF = ff; shooterMotorLeftPIDController.setFF(ff); shooterMotorRightPIDController.setFF(ff); }
    if((max != kMaxOutput) || (min != kMinOutput)) {
      shooterMotorLeftPIDController.setOutputRange(min, max); shooterMotorRightPIDController.setOutputRange(min, max); 
     kMaxOutput = max; kMinOutput = min; }

    if(maxA != maxAccel) { maxAccel = maxA; shooterMotorLeftPIDController.setSmartMotionMaxAccel(maxA, 0); shooterMotorRightPIDController.setSmartMotionMaxAccel(maxA, 0);}

    double setPoint;
    setPoint = SmartDashboard.getNumber("Shooter Set Velocity", 0);
  
    shooterMotorLeftPIDController.setReference(setPoint, ControlType.kVelocity);
    shooterMotorRightPIDController.setReference(-setPoint, ControlType.kVelocity);
  
    SmartDashboard.putNumber("Current Shooter Velocity Left: ", shooterMotorLeftCANEncoder.getVelocity());
    SmartDashboard.putNumber("Current Shooter Velocity Right: ", shooterMotorRightCANEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    //  double newSpeed = SmartDashboard.getNumber("Speed", 0);
    //  if(newSpeed != speed) {
    //    speed = newSpeed;
    //  }

    //  SmartDashboard.putNumber("Current Shooter Velocity Left: ", shooterMotorLeftCANEncoder.getVelocity());
    //  SmartDashboard.putNumber("Current Shooter Velocity Right: ", shooterMotorRightCANEncoder.getVelocity());
    //  PIDSetup();
      
    // SmartDashboard.putNumber("Current Shooter Velocity Left: ", shooterMotorLeftCANEncoder.getVelocity());
    // SmartDashboard.putNumber("Current Shooter Velocity Right: ", shooterMotorRightCANEncoder.getVelocity());
  }
}

