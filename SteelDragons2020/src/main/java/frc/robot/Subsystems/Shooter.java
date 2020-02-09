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


public class Shooter extends SubsystemBase {
  private final Spark shooterMotorLeft;
  private final Spark shooterMotorRight;

  public Shooter() {
    shooterMotorLeft = new Spark(7);
    shooterMotorRight = new Spark(4);
  }

  public void stop() {
    setSpeed(0.0);
  }

  public void forward() {
    setSpeed(-0.7);
  }

  public void setSpeed(double speed){
    shooterMotorRight.set(speed);
    shooterMotorLeft.set(-speed);
  }

  public void PIDSetup() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

