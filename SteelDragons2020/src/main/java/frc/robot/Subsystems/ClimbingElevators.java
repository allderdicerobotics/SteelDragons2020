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

public class ClimbingElevators extends SubsystemBase {
  /**
   * Creates a new ClimbingElevators.
   */
  private VictorSPX elevatorLeft; 
  private VictorSPX elevatorRight;

  public ClimbingElevators() {
    elevatorLeft = new VictorSPX(13);
    elevatorRight = new VictorSPX(15);

    elevatorLeft.configFactoryDefault();
    elevatorRight.configFactoryDefault();

    elevatorLeft.setNeutralMode(NeutralMode.Brake);
    elevatorRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed) {
    elevatorLeft.set(ControlMode.PercentOutput, speed);
    elevatorRight.set(ControlMode.PercentOutput, speed);
  }

  public void goUp() {
    setSpeed(0.3);
  }

  public void goDown() {
    setSpeed(-0.5);
  }

  public void stop() {
    setSpeed(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
