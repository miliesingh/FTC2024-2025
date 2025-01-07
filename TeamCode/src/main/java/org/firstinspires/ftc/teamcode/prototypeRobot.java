package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="prototypeRobot", group="Linear Opmode")
public class prototypeRobot extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;
    private ServoImplEx intakeArmServo = null;

    PwmControl.PwmRange armRange = new PwmControl.PwmRange(1000, 2000);

    private Servo intakeServo = null;
    private Servo intakeWristServo = null;
    private Servo clawServo = null;
    private Servo clawWristServo = null;

    int intakeArmServoCount = 0;
    int intakeServoCount = 0;
    int intakeWristCount = 0 ;
    int clawCount = 0;
    int clawWristServoCount = 0;

    private ElapsedTime runtime = new ElapsedTime();

    // I used this function to try a preset, but it doesn't actually work
    public void LINEAR_SLIDE_DRIVE(float distance_in_in, double power) {
        float ticksPerInch = 450.149432158f;
        float f_ticks = ticksPerInch * distance_in_in;
        int ticks = Math.round(f_ticks);
        //753.1 ticks per revolution
        //1.673 in per revolution (circumference)
        //450.149432158 ticks per in
        if (power > 0) {
            //go up
            linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
            linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            //go down
            linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
            linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setPower(power);
        linSlideL.setTargetPosition(ticks);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setPower(power);
        linSlideR.setTargetPosition(ticks);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("slide", "> is running to position");
        telemetry.update();
        while ((linSlideR.getCurrentPosition() <= linSlideR.getTargetPosition() - 50) && linSlideL.getCurrentPosition() <= (linSlideL.getTargetPosition() - 50)) {
            //Wait until job is finished
            telemetry.addData("slide", "> is strafing to position");
            telemetry.addData("ticks", ">" +
                    linSlideR.getCurrentPosition() + " need to get to " +
                    linSlideR.getTargetPosition());
            telemetry.addData("slide", "> is strafing to position");
            telemetry.addData("ticks", ">" + linSlideL.getCurrentPosition() + " need to get to " + linSlideL.getTargetPosition());
            telemetry.update();
        }

    }
    // preset to move the slides to the blocks height on the wall
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
            intakeWristServo.setPosition(0.5);
        }
        if(intakeWristCount % 2 == 1){
            intakeWristServo.setPosition(Servo.MIN_POSITION);
        }

    }

    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.addData("clawCount", clawCount);
        telemetry.update();

        // setting all of the hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        linSlideL=  hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        intakeArmServo = hardwareMap.get(ServoImplEx.class, "intakeArmServo");
        intakeArmServo.setPwmRange(armRange);
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeWristServo.setPosition(0);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            // Show the elapsed game time and wheel power.
            // telemetry.addData(countPixel);
            //telemetry.update();
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

            if (max > 1.5) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower*1.1);
            rightFront.setPower(rightFrontPower*1.1);
            leftBack.setPower(leftBackPower*1.1); // had to fix both backs to drive
            rightBack.setPower(rightBackPower*1.1);
            // adds precesion mode when bumper pressed
            //changed to try to make it faster


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("intake wrust", intakeWristCount);
            telemetry.update();

            if(gamepad1.right_trigger > 0)
            {// if the right trigger is pressed, the slides move up
                linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideL.setPower(0.7);
                linSlideR.setPower(0.7);
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

        }
    }
}
