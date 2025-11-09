package org.firstinspires.ftc.teamcode;


public class StopLauncher extends Action {
	Auto auto;

	public StopLauncher(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.launcher.SetVelocity(0);
	}

	public boolean isDone() {
		return true;
	}
}
