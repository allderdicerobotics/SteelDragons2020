/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Tube;;

public class IntakeIn extends CommandBase {

  private Intake intake;
  private Tube tube;
  private boolean isAuto;
  
  public IntakeIn(boolean isAuto) {
    this.intake = RobotContainer.intake;
    this.tube = RobotContainer.tube;
    addRequirements(this.intake);

    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.spinStop();
    this.intake.bottom();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isAuto) {
      this.intake.spinOnDriveSpeed();
    } else {
      this.intake.spinIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.spinStop();
    this.intake.top();
    this.tube.bottomPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
