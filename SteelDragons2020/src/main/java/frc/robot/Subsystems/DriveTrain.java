/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  public PIDController alignDTRaspberry;

  double kPLL, kILL, kDLL;


  public DriveTrain() {
    kPLL = 0.02;
    kILL = 0.00;
    kDLL = 0.000;
    //kFFLL = 0.0;
    // SmartDashboard.putNumber("DriveTrain P", kPLL);
    // SmartDashboard.putNumber("DriveTrain I", kILL);
    // SmartDashboard.putNumber("DriveTrain D", kDLL);
    //SmartDashboard.putNumber("DriveTrain FF", kFFLL);

    frontLeft = new CANSparkMax(Constants.CAN_DRIVETRAIN_FRONTLEFT, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.CAN_DRIVETRAIN_BACKLEFT, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.CAN_DRIVETRAIN_FRONTRIGHT, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.CAN_DRIVETRAIN_BACKRIGHT, MotorType.kBrushless);

    frontLeft.setOpenLoopRampRate(0.4);
    backLeft.setOpenLoopRampRate(0.4);
    frontRight.setOpenLoopRampRate(0.4);
    backRight.setOpenLoopRampRate(0.4);

    leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
    rightMotors = new SpeedControllerGroup(frontRight, backRight);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    alignDT = new PIDController(kPLL, kILL, kDLL);
    alignDTRaspberry = new PIDController(0.006, 0, 0);
  }

  public void arcadeDrive(double throttle, double steer) {
    differentialDrive.arcadeDrive(-throttle, steer);
  }

  public void stop() {
    differentialDrive.tankDrive(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // double newP = SmartDashboard.getNumber("DriveTrain P", 0);
    // double newI = SmartDashboard.getNumber("DriveTrain I", 0);
    // double newD = SmartDashboard.getNumber("DriveTrain D", 0);
    // //double newFF = SmartDashboard.getNumber("DriveTrain FF", 0);

    // if(newP != kPLL) { kPLL = newP; }
    // if(newI != kILL) { kILL = newI; }
    // if(newD != kDLL) { kDLL = newD; }
    // //if(newD != kFFLL) { kFFLL = newFF; }
    // alignDT.setPID(kPLL, kILL, kDLL);

    // This method will be called once per scheduler run
  }
}
