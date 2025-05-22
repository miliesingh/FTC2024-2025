package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.KeyStore;

@TeleOp(name="motorTest", group="Linear Opmode")
public class andrewBeingWhiney extends LinearOpMode{

    private DcMotor arm = null;

    public void runOpMode(){

        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.right_trigger > 0){
                arm.setDirection(DcMotorSimple.Direction.FORWARD);
                arm.setPower(0.5);
            } else if (gamepad1.left_trigger > 0) {
                arm.setDirection(DcMotorSimple.Direction.REVERSE);
                arm.setPower(0.5);
            }
            else {
                arm.setPower(0);
            }

        }
    }

    }
