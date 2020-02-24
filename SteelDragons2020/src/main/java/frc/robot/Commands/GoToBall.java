/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class GoToBall extends CommandBase {

  private double[] raspberryValues;
  private DriveTrain driveTrain;
  private double radiusthreshold = 50;
  boolean isAuto;
  public GoToBall(boolean isAuto) {
    driveTrain = RobotContainer.driveTrain;
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get values from pi
    raspberryValues = RobotContainer.getRaspberryValues();
    double currentXValue = raspberryValues[1]-80; //get x coord
    boolean valid = raspberryValues[0] == 1;
    System.out.println(valid);

    //If targets exist
    if(valid) {
      double steercmd = this.driveTrain.alignDTRaspberry.calculate(currentXValue); //Calc PID
      //Limit steercmd
      if(steercmd > 1.0) { steercmd = 1.0; }
      if(steercmd < -1.0) { steercmd = -1.0; }
      this.driveTrain.arcadeDrive(0.0, -steercmd); //Drive
      //System.out.println(currentXValue);
    } else {
      this.driveTrain.arcadeDrive(0.0, 0.3); //If no target, turn
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveTrain.arcadeDrive(0.0, 0.0); //Stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    raspberryValues = RobotContainer.getRaspberryValues();
    if (!this.isAuto) {
      if (!RobotContainer.driver.getRawButton(Constants.kButtonX)) {
        return true;
      }
    }

    //If close enough to balls, stop and quit 
    double currentRValue = raspberryValues[3];
    if (currentRValue >= radiusthreshold) {
      return true;
    }
    return false;
  }
}
