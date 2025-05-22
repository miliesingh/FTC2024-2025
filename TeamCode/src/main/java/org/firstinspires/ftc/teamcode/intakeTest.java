package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "intakeTestForAutomatic", group = "Teleop")
//@Disabled
public class intakeTest extends LinearOpMode {

    private DistanceSensor intakeDistance;

    private Servo intakeServo = null;

    private Servo intakeWristServo = null;
    private ServoImplEx intakeArmServo = null;

    PwmControl.PwmRange armRange = new PwmControl.PwmRange(1000, 2000);


    @Override
    public void runOpMode() {

        intakeDistance = hardwareMap.get(DistanceSensor.class, "intakeDistance");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeArmServo = hardwareMap.get(ServoImplEx.class, "intakeArmServo");
        intakeArmServo.setPwmRange(armRange);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.right_bumper){
                intakeArmServo.setPosition(0.0);
            } else if ( gamepad1.left_bumper){
                intakeArmServo.setPosition(1.0);
            } else {
                intakeArmServo.setPosition(0.5);
            }


            if (gamepad1.right_trigger > 0 && intakeDistance.getDistance(DistanceUnit.INCH) > 2){
                intakeServo.setPosition(1.0);
            } else if (gamepad1.left_trigger > 0){
                intakeServo.setPosition(0.0);

            } else {
                intakeServo.setPosition(0.5);
            }
            if (gamepad1.a){
                intakeWristServo.setPosition(1.0);
            }
            if (gamepad1.b){
                intakeWristServo.setPosition(0.0);
            }
// put it so you can go down with the thing in
            if (intakeDistance.getDistance(DistanceUnit.INCH) < 2 && gamepad1.left_trigger == 0){
                intakeServo.setPosition(0.5);
                intakeWristServo.setPosition(0.0);
            }
            telemetry.addData("Distance", intakeDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();



        }
    }

}