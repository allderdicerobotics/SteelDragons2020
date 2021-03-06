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
import frc.robot.Subsystems.TubeBelts;

public class BeltDownForTime extends CommandBase {
  /**
   * Creates a new BeltDownForTime.
   */
  TubeBelts tubeBelts;
  
  private double startTime = -1.0;
  private boolean isAuto;

  public BeltDownForTime(boolean isAuto) {
    this.tubeBelts = RobotContainer.tubeBelts;
    addRequirements(this.tubeBelts);

    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    this.tubeBelts.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tubeBelts.down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.tubeBelts.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isAuto) {
      if(!RobotContainer.driver.getRawButton(Constants.kButtonA)) {
        return true;
      }
    }
    return (Timer.getFPGATimestamp() >= Constants.AUTO_TUBE_BELTS_DOWN_WAIT_TIME + startTime);
  }
}
