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

public class GoToLLTarget extends CommandBase {
  /**
   * Creates a new GoToLLTarget.
   */

  private DriveTrain drivetrain;
  private double[] limeLightValues;
  boolean doneall;
  
  public GoToLLTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = RobotContainer.driveTrain;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeLightValues = RobotContainer.getLimeLightValues();
    doneall = limeLightValues[0] == 1;
    if (!doneall) {
      drivetrain.arcadeDrive(0, 0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!(RobotContainer.operator.getRawButton(Constants.kButtonA))) {
      return true;
    }
    return doneall;
  }
}
