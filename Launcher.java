package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BotConfig;
import java.util.List;

public class Launcher {

	private LinearOpMode auto;

	public DcMotorEx launcher;


	public Intake(LinearOpMode auto) {
		this.auto = auto;
		
		this.intake = auto.hardwareMap.get(DcMotorEx.class, BotConfig.LAUNCHER_NAME);
	}
  
  
	public void SetVelocity(int velocity) {
		launcher.setVelocity(velocity);
	}


	public void Spin() {
		this.SetVelocity(BotConfig.LAUNCHER_VELOCITY);
	}


	public boolean isBusy() {
		return launcher.isBusy();
	}
	
	public int getVelocity() {
		return launcher.getVelocity();
	}
}