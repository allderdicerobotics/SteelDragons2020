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

  double p, i, d;


  public DriveTrain() {
    p = 0.029;
    i = 0.07;
    d = 0.003;
    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);

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

    alignDT = new PIDController(p, i, d);
  }

  public void arcadeDrive(double throttle, double steer) {
    differentialDrive.arcadeDrive(-throttle, steer);
  }

  public void stop() {
    differentialDrive.tankDrive(0.0, 0.0);
  }

  @Override
  public void periodic() {
    double newP = SmartDashboard.getNumber("P", 0);
    double newI = SmartDashboard.getNumber("I", 0);
    double newD = SmartDashboard.getNumber("D", 0);

    if(newP != p) { p = newP; }
    if(newI != i) { i = newI; }
    if(newD != d) { d = newD; }
    alignDT.setPID(p, i, d);


    // This method will be called once per scheduler run
  }
}
