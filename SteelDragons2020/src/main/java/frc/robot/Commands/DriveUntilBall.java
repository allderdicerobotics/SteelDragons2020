/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;

public class DriveUntilBeamBreak extends CommandBase {
  /**
   * Creates a new DriveUntilBeamBreak.
   */

  private DriveTrain drivetrain;
  DigitalInput beamBreakSensor;
  boolean alldone = false;

  public DriveUntilBeamBreak() {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = RobotContainer.driveTrain;
    beamBreakSensor = new DigitalInput(5);
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Drive if not beam break goes off
    if (!(beamBreakSensor.get())) {
      drivetrain.arcadeDrive(0, 0);
      alldone = true;
    } else {
      drivetrain.arcadeDrive(0.5,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0, 0.0); //Stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alldone; //Stop when beam break
  }
}
