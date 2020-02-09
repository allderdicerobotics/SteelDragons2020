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


public class Tube extends SubsystemBase {
  private Spark beltLeft;
  private Spark beltRight;


  public Tube() {
    beltLeft = new Spark(Constants.PWM_TUBE_BELT_LEFT);
    beltRight = new Spark(Constants.PWM_TUBE_BELT_RIGHT);
  }

  //TUBE BELTS
  public void beltUp() {
    beltLeft.set(-Constants.BELT_SPEED_UP);
    beltRight.set(-Constants.BELT_SPEED_UP);
  }
  public void beltDown() {
    beltLeft.set(Constants.BELT_SPEED_UP);
    beltRight.set(Constants.BELT_SPEED_UP);
  }

  public void beltStop() {
    beltLeft.set(0.0);
    beltRight.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
