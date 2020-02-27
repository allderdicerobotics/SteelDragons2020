/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Tube;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private Shooter shooter;
  private Tube tube;
  private boolean isAuto;

  public Shoot(boolean isAuto) {
    this.shooter = RobotContainer.shooter;
    this.tube = RobotContainer.tube;
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter, this.tube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.normalSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.stop();
    RobotContainer.getRidOfAllBalls();
    this.tube.bottomPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!isAuto) {
      System.out.println("isauto" + isAuto);
      if (!RobotContainer.driver.getRawButton(Constants.kButtonA)) {
        return true;
      }
    }
    return false;
  }
}
