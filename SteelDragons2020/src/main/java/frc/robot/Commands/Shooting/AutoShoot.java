/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.SetDriveTraintoXValue;
import frc.robot.Commands.Shooting.BeltUp;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoShoot.
   */
  private boolean isAuto;
  private boolean fast;

  public AutoShoot(boolean isAuto, boolean fast) {
    this.isAuto = isAuto;
    this.fast = fast;

    addCommands(
      new ParallelCommandGroup(
        new SetDriveTraintoXValue(0.0, this.isAuto),
        new AlignTubeWithTarget(this.isAuto),
        new BeltDownForTime(this.isAuto)
      ),
      new ParallelCommandGroup(
        new AlignTubeWithTarget(this.isAuto),
        new BeltUp(this.isAuto, Constants.kButtonA, true, this.fast),
        new Shoot(this.isAuto, Constants.kButtonA, true, this.fast)
      )
    );
  }
}
