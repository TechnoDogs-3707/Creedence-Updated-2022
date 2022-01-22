/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3707.Creedence.commands.autonomous;

import org.usfirst.frc3707.Creedence.Robot;
import org.usfirst.frc3707.Creedence.commands.hatch.thrustBackwardCommand;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class conditionalThrustBackward extends ConditionalCommand {
  public conditionalThrustBackward() {
    super(new thrustBackwardCommand(), new blankCommand());
  }

  @Override
  protected boolean condition() {
    return Robot.hatchSubsystem.getButton1() || Robot.hatchSubsystem.getButton2();
  }
}
