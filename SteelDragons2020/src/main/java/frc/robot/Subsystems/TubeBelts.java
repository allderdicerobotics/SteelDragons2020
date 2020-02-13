/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TubeBelts extends SubsystemBase {
   
  private Spark beltLeft;
  private Spark beltRight;

  public TubeBelts() {
    beltLeft = new Spark(Constants.PWM_TUBE_BELT_LEFT);
    beltRight = new Spark(Constants.PWM_TUBE_BELT_RIGHT);

  }

  public void up() {
    setSpeed(Constants.BELT_SPEED_UP);
  }

  public void down() {
    setSpeed(-Constants.BELT_SPEED_UP);
  }
  
  public void stop() {
    setSpeed(0.0);
  }

  public void setSpeed(double speed) {
    beltLeft.set(speed);
    beltRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
