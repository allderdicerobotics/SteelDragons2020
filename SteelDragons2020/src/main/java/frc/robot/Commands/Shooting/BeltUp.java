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

public class BeltUp extends CommandBase {

  private TubeBelts tubeBelts;
  private boolean isAuto;
  private double startTime;

  public BeltUp(boolean isAuto) {
    this.tubeBelts = RobotContainer.tubeBelts;
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tubeBelts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.tubeBelts.stop();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tubeBelts.up();
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
    if(isAuto) {
      return (Timer.getFPGATimestamp() >= startTime + 3.0);
    }
    return false;
  }
}
