
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;






@TeleOp(name="odometryTest", group="Linear Opmode")
public class odometryTest extends LinearOpMode {
    private DcMotor backLeft = null;
    private DcMotor  backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;

    private OdometryPod


    static final double MAX_POS = 0.5;
    static final double MIN_POS = 0;

    double position = (MAX_POS - MIN_POS) / 2;


    //variable that holds the amount of time is running
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        backLeft  = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, " back_right");
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //  run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = -1*(axial + lateral + yaw);
            double rightFrontPower = -1*(axial - lateral - yaw);
            double leftBackPower   = -1*(axial - lateral + yaw);
            double rightBackPower  = -1*(axial + lateral - yaw);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // Send calculated power to wheels
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
            // adds precesion mode when bumper pressed


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            //Use the odometry to do stuff
            delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
            delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos
            delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos

            phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth
            delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2
            delta_perp_pos = delta_center_encoder_pos - forward_offset * phi

            delta_x = delta_middle_pos * cos(heading) - delta_perp_pos * sin(heading)
            delta_y = delta_middle_pos * sin(heading) + delta_perp_pos * cos(heading)

            x_pos += delta_x
            y_pos += delta_y
            heading += phi

            prev_left_encoder_pos = left_encoder_pos
            prev_right_encoder_pos = right_encoder_pos
            prev_center_encoder_pos = center_encoder_pos



        }

    }
}