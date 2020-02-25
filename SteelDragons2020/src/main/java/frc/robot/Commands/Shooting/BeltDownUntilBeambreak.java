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

public class BeltDownUntilBeambreak extends CommandBase {
  /**
   * Creates a new BeltDownUntilBeambreak.
   */
  TubeBelts tubeBelts;
  double startTime = -1.0;
  boolean ball = false;
  public BeltDownUntilBeambreak() {
    this.tubeBelts = RobotContainer.tubeBelts;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tubeBelts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    if(RobotContainer.getBeamBreak() && !ball) {
      ball = true;
      startTime = Timer.getFPGATimestamp();
    }
    else if(startTime != -1.0 && Timer.getFPGATimestamp() >= Constants.AUTO_TUBE_BELTS_DOWN_UNTIL_BEAMBREAK_WAIT_TIME + startTime) {
      return true;
    }
    return false;
  }
}
