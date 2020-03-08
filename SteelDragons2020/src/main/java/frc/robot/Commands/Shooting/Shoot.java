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
import frc.robot.Subsystems.Shooter;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private Shooter shooter;

  private int buttonID;
  private double startTime;
  private boolean isDriver;
  private boolean isAuto;
  private boolean fast;

  public Shoot(boolean isAuto, int buttonID, boolean isDriver, boolean fast) {
    this.shooter = RobotContainer.shooter;
    addRequirements(this.shooter);

    this.isAuto = isAuto;
    this.buttonID = buttonID;
    this.isDriver = isDriver;
    this.fast = fast;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.stop();
    startTime = Timer.getFPGATimestamp();
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isAuto) {
      if(isDriver) {
        if(!RobotContainer.driver.getRawButton(this.buttonID)) {
          return true;
        }
      } else {
        if (!RobotContainer.operator.getRawButton(this.buttonID)) {
          return true;
        }
      }
    } else {
      if(this.fast) {
        return (Timer.getFPGATimestamp() >= startTime + 4.0);
      } else {
        return (Timer.getFPGATimestamp() >= startTime + 2.5);
      }
    }
    return false;
  }
}
