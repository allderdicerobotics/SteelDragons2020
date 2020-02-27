/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Tube;

public class TubeTop extends CommandBase {
  /**
   * Creates a new TubeTop.
   */
  Tube tube;
  private double startTime = -1.0;
  public TubeTop() {
    tube = RobotContainer.tube;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.tube.topPosition();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() >= startTime + 1.0);
  }
}
