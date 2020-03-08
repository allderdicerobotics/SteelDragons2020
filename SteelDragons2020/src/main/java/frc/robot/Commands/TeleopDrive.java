/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveTrain;


public class TeleopDrive extends CommandBase {

   private DriveTrain driveTrain;

  public TeleopDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = Constants.DRIVETRAIN_THROTTLE_SCALE * RobotContainer.driver.getRawAxis(Constants.kLeftStickY);
    double steer = Constants.DRIVETRAIN_STEER_SCALE * RobotContainer.driver.getRawAxis(Constants.kRightStickX);
    if(RobotContainer.driver.getRawButton(Constants.kButtonLeftBumper)) {
      throttle *= 0.6;
      steer *= 0.8;
    }
    this.driveTrain.curvatureDrive(throttle, steer, RobotContainer.driver.getRawButton(Constants.kButtonRightBumper));
    // SmartDashboard.putNumber("joystick", RobotContainer.driver.getRawAxis(Constants.kRightStickX));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
