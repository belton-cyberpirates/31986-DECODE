package org.firstinspires.ftc.teamcode;


public class SpinIntake extends Action {
	Auto auto;
	int speed;

	public SpinIntake(Auto auto, int speed) {
		this.auto = auto;
		this.speed = speed;
	}

	public void onStart() {
		this.auto.intake.setIntakeSpeed(speed);
	}

	public boolean isDone() {
		return true;
	}
}
