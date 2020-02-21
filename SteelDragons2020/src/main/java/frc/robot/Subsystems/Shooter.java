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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotorLeft;
  private CANPIDController shooterMotorLeftPIDController;
  private CANEncoder shooterMotorLeftCANEncoder;

  private final CANSparkMax shooterMotorRight;
  private CANPIDController shooterMotorRightPIDController;
  private CANEncoder shooterMotorRightCANEncoder;

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxAccel;
  private double optimalLinearVelocity;

  public Shooter() {
    shooterMotorLeft = new CANSparkMax(Constants.CAN_SHOOTER_LEFT, MotorType.kBrushless);
    // shooterMotorLeftPIDController = shooterMotorLeft.getPIDController();
    // shooterMotorLeftCANEncoder = shooterMotorLeft.getEncoder();

    shooterMotorRight = new CANSparkMax(Constants.CAN_SHOOTER_RIGHT, MotorType.kBrushless);
    // shooterMotorRightPIDController = shooterMotorRight.getPIDController();
    // shooterMotorRightCANEncoder = shooterMotorRight.getEncoder();

    shooterMotorLeft.setIdleMode(IdleMode.kBrake);
    shooterMotorRight.setIdleMode(IdleMode.kBrake);

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kIz = 0.0;
    kFF = 0.0;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;

    maxAccel = 0.0;

    // motorPIDSetup(shooterMotorRightPIDController);
    // motorPIDSetup(shooterMotorLeftPIDController);

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
    controller.setIZone(kIz);
    controller.setFF(kFF);
    controller.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void colorWheelRotate() {
    DCSetSpeed(0.03, -0.03);
  }

  // //REGULAR
  // public void stop() {
  //   setSpeed(0.0);
  // }

  // public void maxSpeed() {
  //   setSpeed(1000);
  // }

  // public void setSpeed(double speed){
  //   shooterMotorLeftPIDController.setReference(speed, ControlType.kSmartVelocity);
  // }

  //DC Speed
  public void DCShootOut() {
    DCSetSpeed(Constants.SHOOTER_SPEED, Constants.SHOOTER_SPEED);
  }

  public void DCSetZero() {
    DCSetSpeed(0.0, 0.0);
  }

  public void DCSetSpeed(double leftSpeed, double rightSpeed) {
    shooterMotorRight.set(rightSpeed);
    shooterMotorLeft.set(leftSpeed);
  }

  public double getOptimalVelocity() {
    double xDist = RobotContainer.getDistanceFromTarget()/12;
    optimalLinearVelocity = (0.00001215593 * Math.pow(xDist, 4)) + (-0.001258 * Math.pow(xDist, 3)) + (0.050136 * Math.pow(xDist, 2)) + (0.435749 * xDist) +17.446296;
    return optimalLinearVelocity;
  }

  public void PIDSetup() {
  //   //Use this only for tuning PID values and testing
  //   double p = SmartDashboard.getNumber("Shooter P", 0);
  //   double i = SmartDashboard.getNumber("Shooter I", 0);
  //   double d = SmartDashboard.getNumber("Shooter D", 0);
  //   double iz = SmartDashboard.getNumber("Shooter I Zone", 0);
  //   double ff = SmartDashboard.getNumber("Shooter Feed Forward", 0);
  //   double max = SmartDashboard.getNumber("Shooter Max Output", 0);
  //   double min = SmartDashboard.getNumber("Shooter Min Output", 0);

  //   double maxA = SmartDashboard.getNumber("Shooter Max Accel", 0);

  //   if(p != kP) { kP = p; shooterMotorPIDController.setP(p); }
  //   if(i != kI) { kI = i; shooterMotorPIDController.setI(i); }
  //   if(d != kD) { kD = d; shooterMotorPIDController.setD(d); }
  //   if(iz != kIz) { kIz = iz; shooterMotorPIDController.setIZone(iz);  }
  //   if(ff != kFF) { kFF = ff; shooterMotorPIDController.setFF(ff);}
  //   if(max != kMaxOutput) || (min != kMinOutput) {
  //    shooterMotorPIDController.setOutputRange(min, max);
  //    kMaxOutput = max; kMinOutput = min; }

  //   if(maxA != maxAccel) { maxAccel = maxA; shooterMotorPIDController.setSmartMotionMaxAccel(maxA, 0);}

  //   double setPoint;
  //   setPoint = SmartDashboard.getNumber("Shooter Set Velocity", 0);
  
  //   shooterMotorPIDController.setReference(setPoint, ControlType.kSmartVelocity);
  
  //   SmartDashboard.putNumber("Current Shooter Velocity: ", shooterMotorCANEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

