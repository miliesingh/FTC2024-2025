package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import static java.lang.Thread.interrupted;

/**
 * FoundationHook spec:
 */
public class ComboGrabber extends Logger<ComboGrabber> implements Configurable {

    final private CoreSystem core;

    private AdjustableServo arm;
    private DcMotorEx slider;
    private AdjustableServo grabber;


    private final double SLIDER_POWER = 0.5;
    private final double SLIDER_SPEED = 1000;
    private final int SLIDER_POS_HIGH = 920;
    private final int SLIDER_POS_HIGHER = 1500;
    private final int SLIDER_POS_INIT = 0;
    private final int SLIDER_POS_LOW = 50;
    private final int SLIDER_POS_MAX = 1733;
    private final int SLIDER_POS_RING = 325;

    private final double ARM_UP = 0.48;
    private final double ARM_INIT = 0.38;
    private final double ARM_DOWN = 0.84;
    private final double ARM_COLLECT_RING = 0.63;

    private final double GRABBER_OPEN = 0.58;
    private final double GRABBER_CLOSE = 0.845;
    private final double GRABBER_INIT = GRABBER_CLOSE;

    private boolean sliderIsLow = true;
    private boolean armIsLow = false;
    private boolean grabberIsClosed = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "topWobbleGoalGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public ComboGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        servoInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        grabber = new AdjustableServo(0, 1).configureLogging(
                logTag + ":topWobbleGoalGrabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);

        slider = configuration.getHardwareMap().get(DcMotorEx.class, "slider");
        if (slider != null) {
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        arm = new AdjustableServo(0, 1).configureLogging(
                logTag + ":topWobbleGoalGrabber", logLevel
        );
        arm.configure(configuration.getHardwareMap(), "arm");
        configuration.register(arm);
        configuration.register(this);
        servoInit();
    }

    public void servoInit() {
        if (grabber!=null)
            grabber.setPosition(GRABBER_INIT);
        if (arm!=null)
            arm.setPosition(ARM_INIT);
        sliderIsLow = false;
        grabberIsClosed = true;
        // configuration.register(this);
    }

    public void sliderStop() {
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (slider==null) return;
        slider.setPower(0);
    }

    public Progress slideToPos(int pos) {
        if (slider==null) return null;
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(SLIDER_SPEED);
        return new Progress() {
            public boolean isDone() {
                if (Math.abs(slider.getCurrentPosition() - slider.getTargetPositionTolerance()) < 50) {
                    return true;
                }
                return !slider.isBusy();
            }
        };
    }

    public void sliderPosHigh() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_HIGH);
        sliderIsLow = false;
    }

    public void slidePosLow() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_LOW);
        sliderIsLow = true;
    }

    public void sliderPosInit() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_INIT);
        sliderIsLow = false;
    }

    public void sliderPosMax() {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideToPos(SLIDER_POS_MAX);
        sliderIsLow = false;
    }

    public void sliderPosAuto() {
        if (slider==null) return;
        if (sliderIsLow)
            sliderPosHigh();
        else
            slidePosLow();
    }

    public void sliderUp(boolean forced) {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = slider.getCurrentPosition();
        if ((pos>=SLIDER_POS_MAX) && !forced) {
            sliderStop();
            return;
        }
        pos = Math.min(pos+50, SLIDER_POS_MAX);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(SLIDER_SPEED);
        // armMotor.setPower(ARM_POWER);
        // armIsDown = false;
    }

    public void sliderDown(boolean forced) {
        if (slider==null) return;
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int pos = slider.getCurrentPosition();
        if (pos<=SLIDER_POS_INIT && !forced) {
            sliderStop();
            return;
        }
        pos=Math.max(pos-50,SLIDER_POS_INIT);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(-SLIDER_SPEED);
        // armMotor.setPower(-ARM_POWER);
        // armIsDown = true;
    }

    public void armUpInc() {
        if (arm==null) return;
        double pos=arm.getPosition()-0.05;
        if (pos<0) pos=0;
        arm.setPosition(pos);
        if (pos<ARM_UP+0.1)
            armIsLow=false;
    }

    public void armDownInc() {
        if (arm==null) return;
        double pos=arm.getPosition()+0.05;
        if (pos>1) pos=1.0;
        arm.setPosition(pos);
        if (pos>ARM_DOWN-0.1)
            armIsLow=true;
    }

    public void armUp() {
        if (arm==null) return;
        arm.setPosition(ARM_UP);
        armIsLow = false;
    }

    public void armDown() {
        if (arm==null) return;
        arm.setPosition(ARM_DOWN);
        armIsLow = true;
    }

    public void armAuto() {
        if (armIsLow) {
            armUp();
        } else {
            armDown();
        }
    }

    public void grabberOpen(){
        grabber.setPosition(GRABBER_OPEN);
        grabberIsClosed = false;
    }

    public void grabberClose(){
        grabber.setPosition(GRABBER_CLOSE);
        grabberIsClosed = true;
    }

    public void grabberAuto(){
        if (grabberIsClosed) {
            grabberOpen();
        } else {
            grabberClose();
        }
    }
    public void releaseWobbleGoalCombo() {
        final String taskName = "release Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        if (slider.getCurrentPosition()>SLIDER_POS_HIGH) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_HIGH);
                }}, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_UP);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }}, taskName);
    }

    public void collectRingCombo(){
        grabWobbleGoalCombo(false);
        while (!TaskManager.isComplete("grab Wobble Goal Combo") && !interrupted()) {
            TaskManager.processTasks();
        }
        moveArm(ARM_COLLECT_RING);
        slideToPos(SLIDER_POS_RING);
    }

    public void grabWobbleGoalCombo(boolean isHigh) {
        final String taskName = "grab Wobble Goal Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_OPEN);
            }}, taskName);
        if (isHigh) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_HIGH);
                }}, taskName);
        } else {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_LOW);
                }}, taskName);
        }
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_DOWN);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveGrabber(GRABBER_CLOSE);
            }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_UP);
            }}, taskName);
        if (isHigh) {
            TaskManager.add(new Task() {
                @Override
                public Progress start() {
                    return slideToPos(SLIDER_POS_HIGHER);
                }}, taskName);
        }
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - arm.getPosition());
        arm.setPosition(position);
        if (position>= ARM_DOWN - 0.1)
            armIsLow=true;
        else {
            armIsLow = false;
        }
        // 1200 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 1200);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    private Progress moveGrabber(double position) {
        double adjustment = Math.abs(position - grabber.getPosition());
        grabber.setPosition(position);
        if (position<= GRABBER_CLOSE + 0.1)
            grabberIsClosed=true;
        else {
            grabberIsClosed = false;
        }
        // 1200 ms per 180 degree
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 1200);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (grabber != null) {
            line.addData("T-Grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }

        if (arm != null) {
            line.addData("Arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
        }

        if (slider != null) {
            line.addData("slider", "%s", new Func<String>() {
                @Override
                public String value() {
                    String s = String.format("pos=%d, pw=%.1f, speed=%.1f", slider.getCurrentPosition(), slider.getPower(),
                            slider.getVelocity());
                    return s;
                }
            });
        }
    }

}



