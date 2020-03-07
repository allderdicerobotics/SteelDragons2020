/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class SetDriveTraintoXValue extends CommandBase {
  /**
   * Creates a new AlignWithTarget.
   */
  private double startTime = -1.0;
  private double[] limeLightValues;
  private double accuracyConstant = 0.7;
  private double PIDInitializeConstant = 11;
  private DriveTrain driveTrain;

  private double commandStartTime = -1.0;
  private double xValue;

  public SetDriveTraintoXValue(double xValue) {
    this.xValue = xValue;
    driveTrain = RobotContainer.driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLightValues = RobotContainer.getLimeLightValues();
    commandStartTime = Timer.getFPGATimestamp();
    this.driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeLightValues = RobotContainer.getLimeLightValues();
    double currentXValue = limeLightValues[1];
    boolean valid = (limeLightValues[0] == 1);

    if (valid) {
      if ((Math.abs(currentXValue - xValue) <= PIDInitializeConstant)) {
        double steercmd = this.driveTrain.alignDT.calculate(currentXValue - xValue);
        steercmd += Math.copySign(0.20, steercmd);
        if (steercmd > 1.0) {
          steercmd = 1.0;
        }
        if (steercmd < -1.0) {
          steercmd = -1.0;
        }
        this.driveTrain.arcadeDrive(0.0, -steercmd);
      } else {
        if (currentXValue < xValue) {
          this.driveTrain.arcadeDrive(0.0, -0.3);
        }
        if (currentXValue > xValue) {
          this.driveTrain.arcadeDrive(0.0, 0.3);
        }
      }
    } else {
      this.driveTrain.arcadeDrive(0.0, 0.0);
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
    boolean valid = (limeLightValues[0] == 1);

    double currentXValue = limeLightValues[1];

    // if we're accurate enough and a timer hasn't been started, start a timer.
    // if we're accurate enough and a timer has been started, we're done if 10 ms
    // has passed
    if ((currentXValue - xValue <= accuracyConstant && currentXValue - xValue >= -accuracyConstant) && valid) {
      if (this.startTime == -1.0) {
        this.startTime = Timer.getFPGATimestamp();
      }
      if (this.startTime != -1.0 && Timer.getFPGATimestamp() >= 0.2 + this.startTime) {
        return true;
      }
    }
    if ((!(currentXValue - xValue <= accuracyConstant && currentXValue - xValue >= -accuracyConstant)) || !valid) {
      startTime = -1.0;
    }
    return false;
  }
}