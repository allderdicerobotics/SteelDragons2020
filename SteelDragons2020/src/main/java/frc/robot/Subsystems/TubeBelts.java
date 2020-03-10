/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TubeBelts extends SubsystemBase {
   
  private VictorSPX beltLeft;
  private VictorSPX beltRight;

  public TubeBelts() {
    beltLeft = new VictorSPX(Constants.CAN_BELT_LEFT);
    beltRight = new VictorSPX(Constants.CAN_BELT_RIGHT);

    beltLeft.setNeutralMode(NeutralMode.Brake);
    beltRight.setNeutralMode(NeutralMode.Brake);
  }

  public void up() {
    setSpeed(Constants.BELT_SPEED_UP);
  }

  public void slowUp() {
    setSpeed(0.3);
  }

  public void down() {
    setSpeed(-Constants.BELT_SPEED_UP);
  }
  
  public void stop() {
    setSpeed(0.0);
  }

  public void setSpeed(double speed) {
    beltLeft.set(ControlMode.PercentOutput, -speed);
    beltRight.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    RobotContainer.putBallCounter();
  }
}
