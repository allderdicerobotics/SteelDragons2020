/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intaking;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.TubeBelts;

public class StoreBalls extends CommandBase {

  TubeBelts tubeBelts;
  DigitalInput beamBreakSensor;
  boolean ball;
  double startTime = -1.0;
  boolean isAuto;

  public StoreBalls(boolean isAuto) {
    this.tubeBelts = RobotContainer.tubeBelts;
    beamBreakSensor = new DigitalInput(5);
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tubeBelts);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.tubeBelts.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!beamBreakSensor.get()) {
      ball = true;
      this.tubeBelts.up();
    }
    else {
      if(ball) {
        startTime = Timer.getFPGATimestamp();
        RobotContainer.addOneBall();
      }
      ball = false;
      if(Timer.getFPGATimestamp() >= Constants.AUTO_INTAKING_EXTRA_BELT_RUN_TIME + startTime) {
        this.tubeBelts.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.tubeBelts.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isAuto) {
      return false;
    }
    return (!RobotContainer.operator.getRawButton(Constants.kBottomRight));  
  }
}
