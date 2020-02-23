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
    this.tube.topPosition();
    double angle = getOptimalAngle();
    //TODO
    //do calculations from Y position to figure out what position to set the tube, then set it.
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

  public double getOptimalAngle() {
    xDist = RobotContainer.getDistanceFromTarget()/12;
    optimalAngle = (0.000009802791 * Math.pow(xDist, 4)) + (-0.001865 * Math.pow(xDist, 3)) + (0.123583 * Math.pow(xDist, 2)) + (-4.051126 * xDist) + 75.484205;
    return optimalAngle;
  }
}
