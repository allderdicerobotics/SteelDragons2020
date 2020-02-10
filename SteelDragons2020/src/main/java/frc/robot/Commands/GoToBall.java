/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class GoToBall extends CommandBase {
  /**
   * Creates a new GoToBall.
   */

  private double[] raspberryValues;
  private DriveTrain driveTrain;
  private double radiusthreshold = 50;

  public GoToBall() {
    driveTrain = RobotContainer.driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    raspberryValues = RobotContainer.getRaspberryValues();
    double currentXValue = raspberryValues[1];
    boolean valid = raspberryValues[0] == 1;
    System.out.println(valid);

    if(valid) {
      double steercmd = this.driveTrain.alignDTRaspberry.calculate(currentXValue);
      if(steercmd > 1.0) { steercmd = 1.0; }
      if(steercmd < -1.0) { steercmd = -1.0; }
      this.driveTrain.arcadeDrive(0.5, -steercmd);
      System.out.println(steercmd);
    } else {
      this.driveTrain.arcadeDrive(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    raspberryValues = RobotContainer.getRaspberryValues();
    if (!RobotContainer.driver.getRawButton(Constants.kButtonA)) {
      return true;
    }

    double currentRValue = raspberryValues[3];
    if (currentRValue >= radiusthreshold) {
      return true;
    }
    return false;
  }
}
