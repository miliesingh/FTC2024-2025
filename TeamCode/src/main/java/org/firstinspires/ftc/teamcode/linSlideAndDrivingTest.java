package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "linearAndDriving", group = "Auto2024")
public class linSlideAndDrivingTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;

    public void linSlidesUp(double power){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(power);
        linSlideR.setPower(power);
    }
    public void linSlidesDown(double power){
        linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideL.setPower(power);
        linSlideR.setPower(power);

    }

    public void linSlidesStay(){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void runOpMode() {
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
        linSlideL = hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        linSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        runtime.reset();

        if(opModeIsActive()){

        }

    }
}
