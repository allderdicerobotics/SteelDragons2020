/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.SetDriveTraintoXValue;
import frc.robot.Commands.Intaking.IntakeAndStore;
import frc.robot.Commands.Shooting.AlignTubeWithTarget;
import frc.robot.Commands.Shooting.AutoShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SixBallTimed extends SequentialCommandGroup {
  /**
   * Creates a new SixBallTimed.
   */
  private double startXValue;

  public SixBallTimed() {
    this.startXValue = RobotContainer.getLimeLightValues()[1];

    addCommands(
      new AutoShoot(true, true),
      new SetDriveTraintoXValue(this.startXValue, true),
      new TubeBottom(),
      new ParallelRaceGroup(
        new IntakeAndStore(true),
        new AutoDrive(2.5, 0.6, true)
      ),
      // new ParallelRaceGroup(
      //   new IntakeAndStore(true),
      //   new WaitTime(1.0)
      // ),
      new AutoDrive(0.2, 1.0, false),
      new ParallelRaceGroup(
        new AlignTubeWithTarget(true),
        new AutoDrive(0.4, 1.0, false)
      ),
      new AutoShoot(true, false)
    );
  }
}
