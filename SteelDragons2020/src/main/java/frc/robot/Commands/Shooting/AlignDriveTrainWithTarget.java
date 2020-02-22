/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class AlignDriveTrainWithTarget extends CommandBase {
  /**
   * Creates a new AlignWithTarget.
   */
  private double startTime = -1.0;
  private double[] limeLightValues;
  private double accuracyConstant = 0.4;
  private double PIDInitializeConstant = 11;
  private DriveTrain driveTrain;

  private double area_min = 0.2;
  private double area_max = 3.5;

  public AlignDriveTrainWithTarget() {
    driveTrain = RobotContainer.driveTrain;
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
    limeLightValues = RobotContainer.getLimeLightValues();
    double currentXValue = limeLightValues[1];
    boolean valid = (limeLightValues[0] == 1);
    //double areaScalar = (Math.abs(((limeLightValues[3]-area_min)/(area_max-area_min))-1))+1;

    if(valid) {
      if((Math.abs(currentXValue) <= PIDInitializeConstant)) {
        double steercmd = this.driveTrain.alignDT.calculate(currentXValue);
        if(steercmd > 1.0) { steercmd = 1.0; }
        if(steercmd < -1.0) { steercmd = -1.0; }
        this.driveTrain.arcadeDrive(0.0, -(steercmd));
      }
      else {
        if(currentXValue < 0.0) { this.driveTrain.arcadeDrive(0.0, -0.3); }
        if(currentXValue > 0.0) {  this.driveTrain.arcadeDrive(0.0, 0.3); }
      }
    } else {
      this.driveTrain.arcadeDrive(0.0, 0.45);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean valid = (limeLightValues[0] == 1);
    if(!RobotContainer.driver.getRawButton(Constants.kButtonA)) {
      return true;
    }

    double currentXValue = limeLightValues[1];

    //if we're accurate enough and a timer hasn't been started, start a timer.
    //if we're accurate enough and a timer has been started, we're done if 10 ms has passed
    if((currentXValue <= accuracyConstant && currentXValue >= -accuracyConstant) && valid){
      if(startTime == -1.0) {
        startTime = Timer.getFPGATimestamp();
      }
      if(startTime != -1.0 && Timer.getFPGATimestamp() >= 0.3 + startTime) {
        return true;
      }
    }
    if(!(currentXValue <= accuracyConstant && currentXValue >= -accuracyConstant)) {
      startTime = -1.0;
    }
    return false;
  }
}
