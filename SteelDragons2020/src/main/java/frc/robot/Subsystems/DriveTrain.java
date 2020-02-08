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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
  private final CANSparkMax frontLeft;
  private final CANSparkMax backLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backRight;

  private final SpeedControllerGroup leftMotors;
  private final SpeedControllerGroup rightMotors;

  private final DifferentialDrive differentialDrive;

  public PIDController alignDT;

  // private CANPIDController leftMotorsPIDController;
  // private CANPIDController rightMotorsPIDController;

  // private CANEncoder leftMotorsCANEncoder;
  // private CANEncoder rightMotorsCANEncoder;

  // private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxAccel;

  // private double currentPositionLeft;
  // private double currentPositionRight;

  public DriveTrain() {
    frontLeft = new CANSparkMax(Constants.CAN_DRIVETRAIN_FRONTLEFT, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.CAN_DRIVETRAIN_BACKLEFT, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.CAN_DRIVETRAIN_FRONTRIGHT, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.CAN_DRIVETRAIN_BACKRIGHT, MotorType.kBrushless);

    // frontLeft.follow(backLeft, false);
    // frontRight.follow(backRight, false);

    leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
    rightMotors = new SpeedControllerGroup(frontRight, backRight);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    alignDT = new PIDController(0.03, 0.000003, 0.003);

    // leftMotorsPIDController = backLeft.getPIDController();
    // rightMotorsPIDController = backRight.getPIDController();

    // leftMotorsCANEncoder = backLeft.getEncoder();
    // rightMotorsCANEncoder = backRight.getEncoder();

    // kP = 0.0;
    // kI = 0.0;
    // kD = 0.0;
    // kIz = 0.0;
    // kFF = 0.0;
    // kMaxOutput = 0.0;
    // kMinOutput = 0.0;

    // maxAccel = 0.0;

    // initiatePID(leftMotorsPIDController);
    // initiatePID(rightMotorsPIDController);
  }

  public void arcadeDrive(double throttle, double steer) {
    differentialDrive.arcadeDrive(throttle, steer);
  }

  public void stop() {
    differentialDrive.tankDrive(0.0, 0.0);
  }

  public double[] getLimeLightValues() {
    double[] values = new double[4];
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically
    values[0] = tv.getDouble(0.0);
    values[1] = tx.getDouble(0.0);
    values[2] = ty.getDouble(0.0);
    values[3] = ta.getDouble(0.0);
    return values;
  }

  // private void setRotationsLeft(double goalPosition) {
  //   leftMotorsPIDController.setReference(goalPosition, ControlType.kSmartMotion);
  // }

  // private void setRotationsRight(double goalPosition) {
  //   rightMotorsPIDController.setReference(goalPosition, ControlType.kSmartMotion);
  // }

  // public void moveLeftForward(double goalMove) {
  //   setRotationsLeft(goalMove + currentPositionLeft);
  // }

  // public void moveRightForward(double goalMove) {
  //   setRotationsRight(goalMove + currentPositionRight);
  // }

  // private void initiatePID(CANPIDController controller) {
  //   controller.setP(kP);
  //   controller.setI(kI);
  //   controller.setD(kD);
  //   controller.setIZone(kIz);
  //   controller.setFF(kFF);
  //   controller.setOutputRange(kMinOutput, kMaxOutput);

  //   controller.setSmartMotionMaxAccel(maxAccel, 0);

  //   // //Use this only for tuning PID values and testing
  //   // SmartDashboard.putNumber("P", kP);
  //   // SmartDashboard.putNumber("I", kI);
  //   // SmartDashboard.putNumber("D" , kD);
  //   // SmartDashboard.putNumber("I Zone", kIz);
  //   // SmartDashboard.putNumber("Feed Forward", kFF);
  //   // SmartDashboard.putNumber("Max Output", kMaxOutput);
  //   // SmartDashboard.putNumber("Min Output", kMinOutput);

  //   // SmartDashboard.putNumber("Max Accel", maxAccel);
  //   // SmartDashboard.putNumber("Set Position", 0);
  //   // SmartDashboard.putNumber("Set Velocity", 0);
  // }

  // public void PIDSetup() {
  // //   //Use this only for tuning PID values and testing
  // //   double p = SmartDashboard.getNumber("P", 0);
  // //   double i = SmartDashboard.getNumber("I", 0);
  // //   double d = SmartDashboard.getNumber("D", 0);
  // //   double iz = SmartDashboard.getNumber("I Zone", 0);
  // //   double ff = SmartDashboard.getNumber("Feed Forward", 0);
  // //   double max = SmartDashboard.getNumber("Max Output", 0);
  // //   double min = SmartDashboard.getNumber("Min Output", 0);

  // //   double maxA = SmartDashboard.getNumber("Max Accel", 0);

  // //   if(p != kP) { kP = p; }
  // //   if(i != kI) { kI = i; }
  // //   if(d != kD) { kD = d; }
  // //   if(iz != kIz) { kIz = iz; }
  // //   if(ff != kFF) { kFF = ff; }
  // //   if(max != kMaxOutput) { kMaxOutput = max; }
  // //   if(min != kMinOutput) { kMinOutput = min; }

  // //   if(maxA != maxAccel) { maxAccel = maxA; }

  // //   initiatePID(leftMotorsPIDController);
  // //   initiatePID(rightMotorsPIDController);

  // //   double setPoint;
  // //   setPoint = SmartDashboard.getNumber("Set Position", 0);
  
  // //   leftMotorsPIDController.setReference(setPoint, ControlType.kSmartMotion);
  // //   rightMotorsPIDController.setReference(setPoint, ControlType.kSmartMotion);
  
  // //   SmartDashboard.putNumber("Current Left Position: ", leftMotorsCANEncoder.getPosition());
  // //   SmartDashboard.putNumber("Current Right Position: ", rightMotorsCANEncoder.getPosition());
  //   currentPositionLeft = leftMotorsCANEncoder.getPosition();
  //   currentPositionRight = rightMotorsCANEncoder.getPosition();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
