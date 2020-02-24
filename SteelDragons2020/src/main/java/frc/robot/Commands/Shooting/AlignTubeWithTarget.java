/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Tube;

public class AlignTubeWithTarget extends CommandBase {

  private double xDist;
  private double optimalAngle;

  private Tube tube;
  private double[] limeLightValues;

  public AlignTubeWithTarget() {
    this.tube = RobotContainer.tube;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLightValues = RobotContainer.getLimeLightValues();
    double yPosition = limeLightValues[2];
    double setPosition = (-1.619075925 * yPosition) + 135.4030406;
    this.tube.setPosition(setPosition);
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
    return true;
  }
}
