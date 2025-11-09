package org.firstinspires.ftc.teamcode;


public class SpinPusher extends Action {
	Auto auto;

	public SpinPusher(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.intake.SpinPusher();
	}

	public boolean isDone() {
		return true;
	}
}
