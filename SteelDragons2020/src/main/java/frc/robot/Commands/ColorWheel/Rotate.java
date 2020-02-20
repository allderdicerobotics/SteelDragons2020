/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.ColorWheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter;

public class Rotate extends CommandBase {
  /**
   * Creates a new Rotate.
   */
  Shooter shooter;
  public Rotate() {
    this.shooter = RobotContainer.shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.DCSetZero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.colorWheelRotate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.DCSetZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
