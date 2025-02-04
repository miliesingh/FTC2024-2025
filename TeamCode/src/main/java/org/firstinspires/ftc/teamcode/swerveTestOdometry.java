package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Locale;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "swerveTestMainOdometry", group = "TeleOp2024")
public class swerveTestOdometry extends LinearOpMode {

    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;
    private ServoImplEx intakeArmServo = null;

    PwmControl.PwmRange armRange = new PwmControl.PwmRange(1000, 2000);

    private Servo intakeServo = null;
    private Servo intakeWristServo = null;
    private Servo clawServo = null;
    private Servo clawWristServo = null;

    GoBildaPinpointDriver odo;

    int intakeArmServoCount = 0;
    int intakeServoCount = 0;
    int intakeWristCount = 0 ;
    int clawCount = 0;
    int clawWristServoCount = 0;

    private ElapsedTime runtime = new ElapsedTime();

    public void moveSlideToPosition(){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(0.7);
        linSlideR.setPower(0.7);
        sleep(333);
        linSlideL.setPower(0.05);
        linSlideR.setPower(0.05);
    }


    // this was the intake for the arm going out and in - never actually use
    public void intakeArmControl(){
        if(intakeArmServoCount % 4 == 0){
            intakeArmServo.setPosition(0.5);
        }
        else if(intakeArmServoCount % 4 == 1){
            intakeArmServo.setPosition(0.55);
            sleep(1000);
            // We need to test this for fully extending
            intakeArmServo.setPosition(0.5);
        }
        else if(intakeArmServoCount % 4 == 2){
            intakeArmServo.setPosition(0.5);
        }
        else if(intakeArmServoCount % 4 == 3){
            intakeArmServo.setPosition(0.45);
            sleep(1000);
            intakeArmServo.setPosition(0.5);
        }
    }
    // test function for the intake servo to spin
    public void intakeServoControl(){
        if(intakeServoCount % 4 == 0){
            intakeServo.setPosition(0.7);
        }
        if(intakeServoCount % 4 == 1){
            intakeServo.setPosition(0.5);
        }
        if(intakeServoCount % 4 == 2){
            intakeServo.setPosition(0.3);
        }
        if(intakeServoCount % 4 == 3){
            intakeServo.setPosition(0.5);
        }
    }
    // function to move the claw wrist up and down
    public void clawWristServoControl(){
        if(clawWristServoCount % 2 == 0){
            clawWristServo.setPosition(Servo.MIN_POSITION);
        }
        if(clawWristServoCount % 2 == 1){
            clawWristServo.setPosition(Servo.MAX_POSITION);
        }
    }
    // function to pinch and unpinch the claw
    public void clawControl(){
        if(clawCount % 2 == 0){
            clawServo.setPosition(Servo.MIN_POSITION);
        }
        if(clawCount % 2 == 1){
            clawServo.setPosition(Servo.MAX_POSITION);
        }
    }
    // function to move the intake wrist up and down
    public void intakeWristServoControl(){
        if(intakeWristCount % 2 == 0){
            intakeWristServo.setPosition(Servo.MAX_POSITION);
        }
        if(intakeWristCount % 2 == 1){
            intakeWristServo.setPosition(Servo.MIN_POSITION);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        linSlideL=  hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        ServoImplEx intakeArmServo = hardwareMap.get(ServoImplEx.class, "intakeArmServo");
        PwmControl.PwmRange armRange = new PwmControl.PwmRange(1000, 2000);

        intakeArmServo.setPwmRange(armRange);
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        odo.setOffsets(65.0, -130); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            odo.bulkUpdate();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.


            double botHeading = odo.getHeading();
            telemetry.update();


            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            telemetry.addData("odometry", "%4.2f", odo.getHeading());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();

            frontLeftMotor.setPower(-frontLeftPower);
            backLeftMotor.setPower(-backLeftPower);
            frontRightMotor.setPower(-frontRightPower);
            backRightMotor.setPower(-backRightPower);


            if(gamepad1.right_trigger > 0)
            {// if the right trigger is pressed, the slides move up
                linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideL.setPower(1);
                linSlideR.setPower(1);
            }else if(gamepad1.left_trigger > 0)
            {
                // left trigger the slides move down
                linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideL.setPower(1.0);
                linSlideR.setPower(1.0);
            }
            else{
                // slides hold if neither are touched
                linSlideL.setPower(0.05);
                linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideR.setPower(0.05);
            }
            if(gamepad2.a){
                // moved the wrist up and down
                intakeWristCount++;
                intakeWristServoControl();
                while (gamepad2.a){
                    // so the buttons dont stick
                    intakeWristCount+=0;
                }
            }
            if(gamepad2.left_trigger > 0){
                // moving the intake out
                intakeArmServo.setPosition(1.0);
            }
            if(gamepad2.right_trigger > 0){
                // moving the intake arm back in
                intakeArmServo.setPosition(0.0);
            }
            if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
                // if neither are touched, then the intake stays where it is
                intakeArmServo.setPosition(0.5);
            }
            if(gamepad2.right_bumper){
                // intake is on
                intakeServo.setPosition(1.0);
            }
            if(gamepad2.left_bumper){
                // intake is out
                intakeServo.setPosition(0.0);
            }
            if(gamepad2.left_bumper == false && gamepad2.right_bumper == false){
                // if neither are touched the intake doesn't move
                intakeServo.setPosition(0.5);
            }
            if(gamepad1.a){
                // opening and closing the claw
                clawCount++;
                clawControl();
                while(gamepad1.a){
                    clawCount+=0;
                }
                telemetry.update();
            }
            if(gamepad1.right_bumper){
                // moving the claw up and down
                clawWristServoCount++;
                clawWristServoControl();
                while(gamepad1.right_bumper){
                    clawWristServoCount+=0;
                }
            }
            if(gamepad1.x){
                // preset to move the slides to the blocks height
                moveSlideToPosition();
            }
            if(gamepad1.left_bumper){
                // preset for the slides to hold at the end of the game
                // ONLY PRESS AT THE END OF THE GAME
                linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideL.setPower(1);
                linSlideR.setPower(1);
                sleep(999999999);
            }
            telemetry.update();
        }
    }
}