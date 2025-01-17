package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name = "distanceDriveSpecimenWall", group = "Auto2024")
public class distanceSensorDriveToSpecimen extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DistanceSensor sensorRange;
    private Servo clawWristServo = null;
    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;
    private Servo clawServo = null;


    public void moveSlideToPosition(){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(0.7);
        linSlideR.setPower(0.7);
        sleep(333);
        linSlideL.setPower(0.05);
        linSlideR.setPower(0.05);
    }


    public void driveForward(double power){
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(-power);
    }
    public void stopRobot(){
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
    }
    public void controlDistance(double distance, double power){
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        double theDistance = sensorRange.getDistance(DistanceUnit.CM);
        // getting the distance
        while (theDistance > distance){
            // while it is out of distance, move forward
            theDistance = sensorRange.getDistance(DistanceUnit.CM);
            driveForward(power);
            telemetry.update();
        }
        stopRobot();
    }
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        linSlideL=  hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();
        runtime.reset();

        double distance = sensorRange.getDistance(DistanceUnit.CM);
        clawWristServo.setPosition(1.0);
        if(opModeIsActive()){
            clawWristServo.setPosition(1.0);
            sleep(333);
            moveSlideToPosition();
            clawServo.setPosition(Servo.MAX_POSITION);
            controlDistance(2, 0.3);
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            moveSlideToPosition();
            sleep(99999999);
        }
    }
}