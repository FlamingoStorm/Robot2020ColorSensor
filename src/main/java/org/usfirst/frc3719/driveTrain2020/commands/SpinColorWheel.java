/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3719.driveTrain2020.commands;

import org.usfirst.frc3719.driveTrain2020.Robot;
import org.usfirst.frc3719.driveTrain2020.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SpinColorWheel extends CommandBase {
  private final WPI_TalonSRX controlPanelSpinner;
  Encoder rS775Encoder;
  int startCoder;

  double distance;
  /**
   * Creates a new spinColorWheel.
   */
 
  public SpinColorWheel(double distance, Encoder rS775Encoder, WPI_TalonSRX controlPanelSpinner) {
    this.controlPanelSpinner = controlPanelSpinner;
    this.rS775Encoder = rS775Encoder;
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int startCoder = rS775Encoder.get();
    
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    controlPanelSpinner.set(.5);
    while(true){
      if(rS775Encoder.get() >= startCoder + (distance * .3)){
        break;
      }

    }
    controlPanelSpinner.set(0);
  }
  public void Stop(){
    controlPanelSpinner.set(0);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
