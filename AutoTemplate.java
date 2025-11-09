package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Name Here"/*, preselectTeleOp="Field Centric (main)"*/)
@Disabled
public class AutoTemplate extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			// Actions Here:
			// Move forward
			new Move(this, 0, 100, 0),

			// Wave arm
			new MoveArm(this, 500, true),
			new MoveArm(this, 50, true),

			// Move back to 0, 0
			new Move(this, 0, 0, 0),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}
