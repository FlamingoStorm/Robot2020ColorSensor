// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc3719.driveTrain2020.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.usfirst.frc3719.driveTrain2020.Robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 *
 */
public class ShooterSpeedDown extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public ShooterSpeedDown() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        addRequirements(Robot.fieldMinipulator);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Robot.fieldMinipulator.setSped(Robot.fieldMinipulator.getSped() - .1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        
        //Robot.fieldMinipulator.Sped(-Robot.oi.xboxControllerJoystick.getTriggerAxis(Hand.kLeft) *25);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {

        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
}
