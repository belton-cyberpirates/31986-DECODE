package org.firstinspires.ftc.teamcode;


public class SpinIntake extends Action {
	Auto auto;

	public SpinIntake(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.intake.SpinIntake();
	}

	public boolean isDone() {
		return true;
	}
}
