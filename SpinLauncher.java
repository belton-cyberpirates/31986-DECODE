package org.firstinspires.ftc.teamcode;


public class SpinLauncher extends Action {
	Auto auto;

	public SpinLauncher(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.launcher.Spin();
	}

	public boolean isDone() {
		return true;
	}
}
