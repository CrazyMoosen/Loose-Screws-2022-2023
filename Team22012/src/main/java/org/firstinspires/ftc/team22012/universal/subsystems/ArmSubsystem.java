package org.firstinspires.ftc.team22012.universal.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmSubsystem {
    private final DcMotor linearSlideMotor;
    private final double TICKS_PER_INCH = 3163d / 38.4d;
    private boolean stopped = false;

    //max is 3163 use ~3100; min is 0
    public ArmSubsystem(DcMotor linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;
        this.linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.linearSlideMotor.setTargetPosition(0);
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPos(double inches) {
        double position = inches * TICKS_PER_INCH;
        this.linearSlideMotor.setTargetPosition((int) position);
        if (position > this.linearSlideMotor.getCurrentPosition()) {
            this.moveUp();
        }
        else {
            this.moveDown();
        }
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stallarm() {
        linearSlideMotor.setPower(0.02);
    }

    public void moveUp(){
        linearSlideMotor.setPower(1);
    }
    public void moveDown(){
        linearSlideMotor.setPower(-1);
    }

    public void runToLowJunction() {
        runToPos(12);
    }

    public void runToMidJunction() {
        runToPos(21);
    }
    public void runToHighJunction() {
        runToPos(32);
    }

    public double getHeight() {
        return linearSlideMotor.getCurrentPosition() / TICKS_PER_INCH;
    }
    public boolean isStopped() {
        return stopped;
    }
    public void stop() {
        this.linearSlideMotor.setPower(0);
    }
}