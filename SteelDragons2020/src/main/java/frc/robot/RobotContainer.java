/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.GoToBall;
import frc.robot.Commands.TeleopDrive;
import frc.robot.Commands.Autonomous.DoNothing;
import frc.robot.Commands.Autonomous.PositionLeft;
import frc.robot.Commands.Autonomous.PositionMiddle;
import frc.robot.Commands.Autonomous.PositionRight;
import frc.robot.Subsystems.DriveTrain;


public class RobotContainer {
    public static final DriveTrain driveTrain = new DriveTrain();

    public static final Joystick driver = new Joystick(0);
    public static final Joystick operator = new Joystick(1);

    private static final String kDefaultAuto = "Do Nothing";
    private static final String kPositionLeft = "PositionLeft";
    private static final String kPositionMiddle = "PositionMiddle";
    private static final String kPositionRight = "PositionRight";

    private String autoSelected;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    
    public Command getAutonomousCommand() {
        autoSelected = autoChooser.getSelected();
        System.out.println("Auto selected: " + autoSelected);
        Command returnCommand;
        switch(autoSelected) {
            case "PositionLeft":
                returnCommand = (new PositionLeft());
                break;
            case "PositionMiddle":
                returnCommand = (new PositionMiddle());
                break;
            case "PositionRight":
                returnCommand = (new PositionRight());
                break;
            default:
                returnCommand = (new DoNothing());
                break;
        }
        return returnCommand;
    }

    public RobotContainer() {
        configureButtonBindings();
        
        driveTrain.setDefaultCommand(new TeleopDrive(driveTrain));

        autoChooser.setDefaultOption("Do Nothing", kDefaultAuto);
        autoChooser.addOption("PositionLeft", kPositionLeft);
        autoChooser.addOption("PositionMiddle", kPositionMiddle);
        autoChooser.addOption("PositionRight", kPositionRight);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void configureButtonBindings() {
        new JoystickButton(driver, Constants.kButtonX)
            .whenActive(new GoToBall(Constants.kButtonX));
    }

    public static double[] getRaspberryValues() {
        double[] values = new double[4];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("FRCvisionpc");
        NetworkTableEntry tx = table.getEntry("pi_tx");
        NetworkTableEntry ty = table.getEntry("pi_ty");
        NetworkTableEntry ta = table.getEntry("pi_ta");
        
        //read values periodically
        values[1] = tx.getDouble(0.0);
        values[2] = ty.getDouble(0.0);
        values[3] = ta.getDouble(0.0);
        values[0] = ((values[1]==-1)&&(values[2]==-1))?0:1;
        return values;
      }
}
