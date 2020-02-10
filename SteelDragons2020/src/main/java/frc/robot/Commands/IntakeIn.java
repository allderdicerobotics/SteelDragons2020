/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Intake;

public class IntakeIn extends CommandBase {
  /**
   * Creates a new IntakeIn.
   */
  Intake intake;
  int buttonid;
  boolean isoperator;
  public IntakeIn(int buttonid, boolean isoperator) {
    this.intake = RobotContainer.intake;
    this.buttonid = buttonid;
    this.isoperator = isoperator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.spinStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intake.spinIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.spinStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.isoperator) {
      return (!RobotContainer.operator.getRawButton(this.buttonid));
    } else {
      return (!RobotContainer.driver.getRawButton(this.buttonid));
    }
  }
}