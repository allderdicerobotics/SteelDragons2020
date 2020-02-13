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

import frc.robot.Commands.TeleopDrive;
import frc.robot.Commands.Autonomous.DoNothing;
import frc.robot.Commands.Autonomous.PositionLeft;
import frc.robot.Commands.Autonomous.PositionMiddle;
import frc.robot.Commands.Autonomous.PositionRight;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Tube;
import frc.robot.Subsystems.TubeBelts;


public class RobotContainer {
    public static final DriveTrain driveTrain = new DriveTrain();
    public static final Tube tube = new Tube();
    public static final Intake intake = new Intake();
    public static final Shooter shooter = new Shooter();
    public static final TubeBelts tubeBelts = new TubeBelts();

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
        //TUBE
        new JoystickButton(operator, Constants.kButtonY)
            .whileActiveContinuous(() -> tube.up());
        new JoystickButton(operator, Constants.kButtonA)
            .whileActiveContinuous(() -> tube.down());

        //INTAKE
        new JoystickButton(driver, Constants.kButtonX)
            .whenPressed(() -> intake.spinIn())
            .whenReleased(() -> intake.spinStop());
        new JoystickButton(driver, Constants.kButtonA)
            .whenPressed(() -> intake.spinOut())
            .whenReleased(() -> intake.spinStop());

        new JoystickButton(driver, Constants.kButtonB)
            .whileActiveContinuous(() -> intake.top());
        new JoystickButton(driver, Constants.kButtonY)
            .whileActiveContinuous(() -> intake.bottom());

        //SHOOTER
        new JoystickButton(operator, Constants.kButtonX)
            .whileActiveContinuous(() -> shooter.maxSpeed())
            .whenInactive(() -> shooter.stop());
    }

    public static double[] getLimeLightValues() {
        double[] values = new double[4];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        
        //read values periodically
        values[0] = tv.getDouble(0.0);
        values[1] = tx.getDouble(0.0);
        values[2] = ty.getDouble(0.0);
        values[3] = ta.getDouble(0.0);
        return values;
      }
}
