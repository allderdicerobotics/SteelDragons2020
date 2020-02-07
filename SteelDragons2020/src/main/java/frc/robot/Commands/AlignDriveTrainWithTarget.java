/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class AlignDriveTrainWithTarget extends CommandBase {
  /**
   * Creates a new AlignWithTarget.
   */
  private double startTime = -1.0;
  private double[] limeLightValues;
  private double accuracyConstant = 1.0;
  private DriveTrain driveTrain;

  public AlignDriveTrainWithTarget() {
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
    limeLightValues = driveTrain.getLimeLightValues();
    double currentXValue = limeLightValues[0];

    //TODO
    //Use a PID controller to calculate turning values to drive on
    //call arcade drive using those values

    //These actions occur once per loop (every 20 ms)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentXValue = limeLightValues[0];

    //if we're accurate enough and a timer hasn't been started, start a timer.
    //if we're accurate enough and a timer has been started, we're done if 10 ms has passed
    if(currentXValue <= accuracyConstant || currentXValue >= -accuracyConstant){
      if(startTime == -1.0) {
        startTime = Timer.getFPGATimestamp();
      }
      if(startTime != -1.0 && Timer.getFPGATimestamp() >= 0.01 + startTime) {
        return true;
      }
    }
    if(!(currentXValue <= accuracyConstant || currentXValue >= -accuracyConstant)) {
      startTime = -1.0;
    }
    return false;
  }
}
