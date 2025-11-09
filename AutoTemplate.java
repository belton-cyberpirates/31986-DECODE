package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Name Here"/*, preselectTeleOp="Your Drive Code Here"*/)
//@Disabled
public class AutoTemplate extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			// Actions Here:
			// Move backward
			new Move(this, 0, -100, 0),

			// Prepare to launch
			new OpenStopper(this),
			new 
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

