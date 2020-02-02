/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
  private final Spark frontLeft;
  private final Spark backLeft;
  private final Spark frontRight;
  private final Spark backRight;

  private final SpeedControllerGroup leftMotors;
  private final SpeedControllerGroup rightMotors;

  private final DifferentialDrive differentialDrive;

  public DriveTrain() {
    frontLeft = new Spark(Constants.PWM_DRIVETRAIN_FRONTLEFT);
    backLeft = new Spark(Constants.PWM_DRIVETRAIN_BACKLEFT);
    frontRight = new Spark(Constants.PWM_DRIVETRAIN_FRONTRIGHT);
    backRight = new Spark(Constants.PWM_DRIVETRAIN_BACKRIGHT);

    leftMotors = new SpeedControllerGroup(frontLeft, backLeft);
    rightMotors = new SpeedControllerGroup(frontRight, backRight);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void arcadeDrive(double throttle, double steer) {
    differentialDrive.arcadeDrive(-throttle, steer);
  }

  public void stop() {
    differentialDrive.tankDrive(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
