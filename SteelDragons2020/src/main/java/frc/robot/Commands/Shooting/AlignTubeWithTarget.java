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
import frc.robot.Subsystems.Tube;

public class AlignTubeWithTarget extends CommandBase {

  private Tube tube;
  private double[] limeLightValues;
  private boolean done = false;

  private double startTime = -1.0;
  private boolean didStartTime = false;
  private boolean isAuto;

  public AlignTubeWithTarget(boolean isAuto) {
    this.tube = RobotContainer.tube;
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeLightValues = RobotContainer.getLimeLightValues();
    boolean valid = (limeLightValues[0] == 1);
    if(valid) {
      double yPosition = limeLightValues[2];
      //double setPosition = (-1.619075925 * yPosition) + 132.4030406;
      double setPosition = ((-2.68836 * yPosition) + 123.903) - 13.5;
      this.tube.setPosition(setPosition);
      done = true;
    } else {
      done = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isAuto) {
      if(!RobotContainer.driver.getRawButton(Constants.kButtonA)) {
        return true;
      }
    }
    if(done && !didStartTime) {
      startTime = Timer.getFPGATimestamp();
      didStartTime = true;
    }
    else if (done && didStartTime && Timer.getFPGATimestamp() >= 0.5 + startTime) {
      return true;
    }
    if(!done) {
      startTime = -1.0;
      didStartTime = false;
    }
    return false;
  }
}
