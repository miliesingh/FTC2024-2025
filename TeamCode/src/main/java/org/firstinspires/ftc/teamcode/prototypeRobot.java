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
    private Servo intakeArmServo = null;
    private Servo intakeServo = null;
    private ServoImplEx intakeWristServo = null;
    PwmControl.PwmRange range = new PwmControl.PwmRange(900, 2100);
    private Servo clawServo = null;
    private Servo clawWristServo = null;

    int intakeArmServoCount = 0;
    int intakeServoCount = 0;
    int intakeWristCount = 0 ;
    int clawCount = 0;
    int clawWristServoCount = 0;

    private ElapsedTime runtime = new ElapsedTime();

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
    public void clawWristServoControl(){
        if(clawWristServoCount % 3 == 0){
            clawWristServo.setPosition(Servo.MIN_POSITION);
        }
        if(clawWristServoCount%3 == 1){
            clawWristServo.setPosition((Servo.MAX_POSITION-Servo.MIN_POSITION)/2);
        }
        if(clawWristServoCount % 3 == 2){
            clawWristServo.setPosition(Servo.MAX_POSITION);
        }
    }
    public void clawControl(){
        if(clawCount % 2 == 0){
            clawServo.setPosition(Servo.MIN_POSITION);
        }
        if(clawCount % 2 == 1){
            clawServo.setPosition(Servo.MAX_POSITION);
        }
    }
    public void intakeWristServoControl(){
        if(intakeWristCount % 2 == 0){
            intakeWristServo.setPosition(Servo.MIN_POSITION);
        }
        if(intakeWristCount % 2 == 1){
            intakeWristServo.setPosition(Servo.MAX_POSITION);
        }
    }

    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        linSlideL=  hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        intakeWristServo = hardwareMap.get(ServoImplEx.class, "intakeWristServo");
        intakeWristServo.setPwmRange(range);
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
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower); // had to fix both backs to drive
            rightBack.setPower(rightBackPower);
            // adds precesion mode when bumper pressed


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("intake wrust", intakeWristCount);
            telemetry.update();

            if(gamepad1.right_trigger > 0)
            {
                linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideL.setPower(0.6);
                linSlideR.setPower(0.6);
            }else if(gamepad1.left_trigger > 0)
            {
                linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideL.setPower(0.9);
                linSlideR.setPower(0.9);
            }
            else{
                linSlideL.setPower(0.05);
                linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
                linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
                linSlideR.setPower(0.05);
            }
            if(gamepad2.a){
            intakeWristServo.setPosition(1);
            }
            if(gamepad2.b){
                intakeWristServo.setPosition(0.0);
            }
            if(gamepad2.right_trigger > 0){
                intakeArmServo.setPosition(0.9);
            }
            if(gamepad2.left_trigger > 0){
                intakeArmServo.setPosition(0.1);
            }
            if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
                intakeArmServo.setPosition(0.5);
            }
            if(gamepad2.left_bumper){
                intakeServo.setPosition(1.0);
            }
            if(gamepad2.right_bumper){
                intakeServo.setPosition(0.0);
            }
            if(gamepad2.x){
                intakeServo.setPosition(0.5);
            }
            if(gamepad1.a){
                clawCount++;
                clawControl();
            }
            if(gamepad1.right_bumper){
                clawWristServoCount++;
                clawWristServoControl();
            }
        }
    }
}
