package org.firstinspires.ftc.teamcode;


public class SpinPusher extends Action {
	Auto auto;

	public SpinPusher(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.intake.SetPusherVelocity(0);
	}

	public boolean isDone() {
		return true;
	}
}
