/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  /**
   * Creates a new AutoTurn.
   */
  private double waitTime;
  private double startTime;
  private DriveTrain driveTrain;
  private boolean left;
  private double speed;

  public AutoTurn(double time, double speed, boolean left) {
    this.waitTime = time;
    this.speed = speed;
    this.left = left;
    this.driveTrain = RobotContainer.driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(left) {
      this.driveTrain.arcadeDrive(0.0, -this.speed);
    }
    if(!left) {
      this.driveTrain.arcadeDrive(0.0, this.speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() >= this.startTime + this.waitTime);
  }
}
