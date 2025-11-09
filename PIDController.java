package org.firstinspires.ftc.teamcode;


public class PIDController {
	double Kp;
	double Ki;
	double Kd;

	double integralSum = 0;
	double lastError = 100;
	
	double lastOutput = 10000;

	public PIDController(double Kp, double Ki, double Kd) {
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
	}

	public double PIDControl(double reference, double state, double delta) {
		double error = reference - state;
		return PIDControl(error, delta);
	}

	public double PIDControlRadians(double reference, double state, double delta) {
		double error = angleWrap(reference - state);
		if (Double.isNaN(error)) {
			throw new RuntimeException("Error is NaN: " + error + ", reference=" + reference + ", state=" + state + ", delta=" + delta);
		}
		return PIDControl(error, delta);
	}



	public double PIDControl(double error, double delta) {
		
		double derivative = (error - lastError) / delta;

		this.lastError = error;
		this.integralSum += error * delta;

		double output = (error * this.Kp) + (derivative * this.Kd) + (this.integralSum * this.Ki);
		
		lastOutput = output;
		return output;
	}
	
	

	public double angleWrap(double radians) {
		while (radians > Math.PI) {
			radians -= 2 * Math.PI;
		}
		
		while (radians < -Math.PI) {
			radians += 2 * Math.PI;
		}

		return radians;
	}
}
