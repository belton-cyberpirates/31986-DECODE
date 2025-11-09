package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Wait extends Action {
	Auto auto;
	double time;
	ElapsedTime timer;

	public Wait(Auto auto, double time) {
		this.auto = auto;
		
		this.time = time;
	}

	public void onStart() {
		this.timer = new ElapsedTime();
	}
	
	public boolean isDone() {
		return this.timer.milliseconds() >= time;
	}
	
	public void process() {
		auto.telemetry.addData("Wait time remaining", time - timer.milliseconds());
	}

}