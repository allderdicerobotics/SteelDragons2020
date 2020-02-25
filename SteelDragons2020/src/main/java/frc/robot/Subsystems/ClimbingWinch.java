/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingWinch extends SubsystemBase {
  /**
   * Creates a new ClimbingWinch.
   */
  private CANSparkMax climbingWinch;

  public ClimbingWinch() {
    climbingWinch = new CANSparkMax(17, MotorType.kBrushless);

    climbingWinch.restoreFactoryDefaults();

    climbingWinch.setIdleMode(IdleMode.kBrake);
  }

  public void setSpeed(double speed) {
    climbingWinch.set(speed);
  }

  public void winchDown() {
    setSpeed(-0.4);
  }

  public void letLoose() {
    setSpeed(0.4);
  }

  public void stop() {
    setSpeed(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
