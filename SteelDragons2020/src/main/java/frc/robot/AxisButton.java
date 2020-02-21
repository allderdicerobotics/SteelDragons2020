/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public class AxisButton extends Trigger {
    
    private GenericHID joystick;
    private int buttonNumber;
    private boolean up;

    public AxisButton(GenericHID joystick, int buttonNumber, boolean up) {
        this.buttonNumber = buttonNumber;
        this.joystick = joystick;
        this.up = up;
    }

    public boolean get() {
        boolean move = false;
        double axis = joystick.getRawAxis(buttonNumber);
        if(axis>0.07 && up){
            move = true;
        }
        else if(axis<-0.07 && !up){
            move = true;
        }
        return move;
    }
}