package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp (name = "Limelight", group  = "Linear OpMode")
public class limelightTest extends LinearOpMode{
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    Limelight3A limelight;

    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        limelight.setPollRateHz(100);
        limelight.start();
        waitForStart();
        while (opModeIsActive()){
            limelight.pipelineSwitch(1);
            LLResult result = limelight.getLatestResult();
            int index = result.getPipelineIndex();

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
            leftFront.setPower(leftFrontPower*1.6);
            rightFront.setPower(rightFrontPower*1.6);
            leftBack.setPower(leftBackPower*1.6); // had to fix both backs to drive
            rightBack.setPower(rightBackPower*1.6);

            if(result != null && result.isValid()){
                Pose3D botpose = result.getBotpose();
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                if (botpose != null){
                    double x= botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("Location", "( x " + x + ", y " + y + ")");
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.addData("index", index);
            telemetry.update();
        }
        limelight.stop();
    }
}
