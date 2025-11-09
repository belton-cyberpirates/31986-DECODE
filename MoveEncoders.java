package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Direction;


public class Move extends Action {
	Auto auto;
	Direction dir;
	double distance;
	ElapsedTime startTime;
	
	public Move(Auto auto, Direction dir, double distance) {
		this.auto = auto;
		
		this.dir = dir;
		this.distance = distance;
	}

	public void onStart() {
		startTime = new ElapsedTime();
		auto.driveMotors.Move(dir, distance);
	}
	
	public boolean isDone() {
		return auto.driveMotors.isDone() || (expectedTime != 0 && startTime.time() >= expectedTime);
	}

}
