package org.firstinspires.ftc.teamcode;


public class WaitForLauncher extends Action {
	Auto auto;


	public WaitForLauncher(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {}

	public boolean isDone() {
		return !this.auto.launcher.isBusy();
	}
}