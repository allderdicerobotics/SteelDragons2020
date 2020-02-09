/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


public class Constants {
	//CAN IDs
    public static final int CAN_DRIVETRAIN_FRONTLEFT = 0;
    public static final int CAN_DRIVETRAIN_BACKLEFT = 1;
    public static final int CAN_DRIVETRAIN_FRONTRIGHT = 2;
    public static final int CAN_DRIVETRAIN_BACKRIGHT = 3;

    public static final int CAN_TUBE = 4;
    public static final int CAN_SHOOTER_LEFT = 5;
    public static final int CAN_SHOOTER_RIGHT = 6;

	public static final int CAN_FOLD_INTAKE = 7;
	public static final int CAN_SPIN_INTAKE = 4;

	//PWM IDs
	public static final int PWM_TUBE_BELT_RIGHT = 6;
	public static final int PWM_TUBE_BELT_LEFT = 5;

	//SPEEDS
    public static final double DRIVETRAIN_THROTTLE_SCALE = 0.6;
    public static final double DRIVETRAIN_STEER_SCALE = 0.5;

    public static final double INTAKE_SPEED_IN = -0.9;
	public static final double INTAKE_SPEED_OUT = 0.9;

	public static final double BELT_SPEED_UP = 0.4;
	public static final double BELT_SPEED_DOWN = 0.4;
	
	//OTHER
    public static final double TUBE_MANUAL_CHANGE = 1.9;

    //Logitech f310
	/* axis mappings */
	public static final int kLeftStickX = 0;
	public static final int kLeftStickY = 1;
	public static final int kLeftTrigger = 2;
	public static final int kRightTrigger = 3;
	public static final int kRightStickX = 4;
	public static final int kRightStickY = 5;
	
	/* button mappings */
	public static final int kButtonA = 1;
	public static final int kButtonB = 2;
	public static final int kButtonX = 3;
	public static final int kButtonY = 4;
	public static final int kButtonLeftBumper = 5;
	public static final int kButtonRightBumper = 6;
	public static final int kButtonBack = 7;
	public static final int kButtonStart = 8;
	public static final int kButtonLeftStick = 9;
	public static final int kButtonRightStick = 10;
	public static final int kPOVDPad = 0;
}
