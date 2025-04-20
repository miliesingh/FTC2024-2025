package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp (name = "Limelight", group  = "Linear OpMode")
public class limelightTest extends LinearOpMode{
    Limelight3A limelight;

    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        waitForStart();
        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            limelight.pipelineSwitch(1);
            int index = result.getPipelineIndex();
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
                    telemetry.addData("Location", "(" + x + ", " + y + ")");
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.addData("index", index);
            telemetry.update();
        }
    }
}
