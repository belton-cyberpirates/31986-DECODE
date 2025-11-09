package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Centric (main)", group="DriveCodes")
public class DriveCode extends LinearOpMode {
    
    // Drive constants
    final double BASE_SPEED = .5;
    final double MAX_BOOST = 0.66; // boost maxes out at an additional 60% of the base speed
    final double STRAFE_MULT = 1.2;
    final double TURN_MULT = 1.2;

    DriveMotors driveMotors;
    
    DcMotorEx intake;
    DcMotorEx pusher;
    DcMotorEx flywheel;

    @Override
    public void runOpMode() throws InterruptedException {
        
        driveMotors = new DriveMotors(this);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        pusher = hardwareMap.get(DcMotorEx.class, "pusher");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Reset yaw when y button pressed so restarting is not needed if it needs a reset
            // if (gamepad1.y) {
            //     driveMotors.odometry.recalibrateIMU();
            // }
            

            // Process classes
            double deltaTime = driveMotors.process();

            
            // P1 variables
            double leftStickXGP1 = gamepad1.left_stick_x;
            double leftStickYGP1 = gamepad1.left_stick_y;
            double rightStickXGP1 = gamepad1.right_stick_x;
            double rightStickYGP1 = gamepad1.right_stick_y;

            // Get the speed the bot would go with the joystick pushed all the way
            double maxSpeed = calcMaxSpeed(gamepad1.right_trigger - gamepad1.left_trigger, BASE_SPEED, MAX_BOOST);
            
            double joystickLength = Math.sqrt( Math.pow(gamepad1.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2) );
            double joystickAngle = -Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
            
            double turnPower = 
            joystickLength > 10 ?
                driveMotors.imuPidController.PIDControlRadians(
                    joystickAngle,
                    driveMotors.heading,
                    deltaTime
                )
            : 
                -gamepad1.right_stick_x;

            // Virtually rotate the joystick by the angle of the robot
            double rotatedX = leftStickXGP1;
                // leftStickXGP1 * Math.cos(driveMotors.heading) -
                // leftStickYGP1 * Math.sin(driveMotors.heading);
            double rotatedY = leftStickYGP1;
                // leftStickXGP1 * Math.sin(driveMotors.heading) +
                // leftStickYGP1 * Math.cos(driveMotors.heading);
            
            // strafing is slower than rolling, bump horizontal speed
            rotatedX *= STRAFE_MULT;
            
            // if (gamepad1.a) {
            //     driveMotors.Move(BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 180);
            // }
            // else {
                // Set the power of the wheels based off the new joystick coordinates
                // y+x+stick <- [-1,1]
                driveMotors.DriveWithPower(
                    ( rotatedY + rotatedX + ( turnPower )) * maxSpeed, // Back left
                    ( rotatedY - rotatedX + ( turnPower )) * maxSpeed, // Front left
                    (-rotatedY - rotatedX + ( turnPower )) * maxSpeed, // Front right
                    (-rotatedY + rotatedX + ( turnPower )) * maxSpeed  // Back right
                );
            // }
            
            // P2 variables
            double leftStickYGP2 = gamepad2.left_stick_y;
            double rightStickYGP2 = gamepad2.right_stick_y;
            
            // Flywheel
            if (gamepad2.a) {
                runFlywheel(4400);
            }
            else if (gamepad2.b) {
                runFlywheel(4000);
            }
            else {
                FlyR.setPower(0);
                FlyL.setPower(0);
            }

            // Banana
            if (gamepad2.dpad_up) {
                Banana.setPosition(0.3);
            } else {
                Banana.setPosition(0.6);
            }
            
            // Intake
            if (gamepad1.right_bumper) {
                Intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                Intake.setPower(-1);
            } else {
                Intake.setPower(0);
            }
        
            // Telemetry
            // Odometry values
            // telemetry.addData("X pos", driveMotors.odometry.getPosX(DistanceUnit.MM));
            // telemetry.addData("Y pos", driveMotors.odometry.getPosY(DistanceUnit.MM));
            // telemetry.addData("Heading", driveMotors.odometry.getHeading(AngleUnit.DEGREES));

            // Turning values
            telemetry.addData("joystickAngle", joystickAngle);
            telemetry.addData("turnPower", turnPower);
            
            telemetry.update();
        }
    }


    /**
     * if boost trigger unpressed, return base_speed,
     * else return base_speed + boost amount
     */
    double calcMaxSpeed(double triggerVal, double BASE_SPEED, double MAX_BOOST) {
        double boostRatio = triggerVal * MAX_BOOST;
        double boostSpeed = boostRatio * BASE_SPEED;
        return BASE_SPEED + boostSpeed;
    }


    private void runFlywheel(int targetRPM){
        double currentVelocity = (((DcMotorEx) FlyL).getVelocity() ); // in ticks/sec
        double ticksPerRev = 28.0; // GoBILDA motors (adjust if using different)
        double currentRPM = (currentVelocity / ticksPerRev) * 60.0;
        
        smoothedRPM = alpha * currentRPM + (1 - alpha) * smoothedRPM;

        // Bang-bang control with hysteresis
        if (smoothedRPM < targetRPM - tolerance) {
            FlyL.setPower(highPower);
            FlyR.setPower(highPower);
        } else if (smoothedRPM > targetRPM + tolerance) {
            FlyL.setPower(lowPower);
            FlyR.setPower(lowPower);
        }
}