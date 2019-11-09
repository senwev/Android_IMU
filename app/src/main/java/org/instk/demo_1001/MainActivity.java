package org.instk.demo_1001;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.GLSurfaceView;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends AppCompatActivity implements SensorEventListener  {

    private final float N2S=1000000000f;

    //Initial Camera Pos/Att
    private float[] INIPos={0,0,1f};
    private float[] INIVel={0,0,0};
    private float[] INICbn={1,0,0,0,1,0,0,0,1};

    private SensorManager mSMan;
    private INS mINS;
    private Kalman mKalman;

    //Debug Text View
    TextView etv;

    //Mode of the program
    //0 = 3Dof (Acc+Compass)
    //1 = 3Dof (Gyro)
    //2 = 6Dof
    //3 = Bias Removal
    int mode=0;
    boolean ZFlag=false, CFlag=false;

    //Most recent sensor data and its timestamp
    float[] dAcc=new float[3];
    float[] dGyro=new float[3];
    float[] dMag=new float[3];
    long etime=0; //Time for the latest sensor data
    long ptAcc=0,ptGyro=0,ptMag=0;	//previous Sample Time

    //Sensor Calibration values
    float[] cBAcc=new float[3];
    float[] cBGyro=new float[3];

    //Periods
    private final long PERBR=(long) (1*N2S);	//Data collection period for bias removal (Nanosec)
    private long tBR=0;
    private final long PERPROP=(long) (1*N2S);	//KF maximum covariance propagation period (Nanosec)
    private long ptProp=0, tProp=0;

    //Temporary variables to be used (used to redeuce the load of GC)
    float[] vr_a=new float[3];
    float[] vr_b=new float[3];
    float[] mx_a=new float[9];


    Button startButton;

    List<float[]> storePos ;

    CanvasView canvasView ;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        canvasView=(CanvasView) findViewById( R.id.canvasview);

        storePos = new ArrayList<>();


        startButton = (Button) findViewById(R.id.button);


        startButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {

                if(motionEvent.getAction() == MotionEvent.ACTION_DOWN)
                {
                    mode=3;
                    return true;
                }
                else if(motionEvent.getAction() == MotionEvent.ACTION_UP)
                {
                    mode=0;
                    canvasView.setDataList(storePos);
                    return true;
                }
                return false;
            }
        });



         etv= (TextView) findViewById(R.id.textView2);

        //Get a reference to sensor manager
        mSMan = (SensorManager)getSystemService(Context.SENSOR_SERVICE);




        mINS=new INS(INIPos, INIVel, INICbn);
        mKalman=new Kalman();
    }

    @Override
    protected void onResume() {
        super.onResume();
        //OpenGl View

        //Sensors
        Sensor sAcc = mSMan.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor sMag = mSMan.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor sGyro = mSMan.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        int rate=SensorManager.SENSOR_DELAY_FASTEST;

        mSMan.registerListener(this, sAcc, rate);
        if (sMag!=null) mSMan.registerListener(this, sMag, rate);
        if (sGyro!=null)
            mSMan.registerListener(this, sGyro, rate);
        else
            etv.setText("## You don't have a gyro ##");
    }

    @Override
    protected void onPause() {
        // Ideally a game should implement onResume() and onPause()
        // to take appropriate action when the activity looses focus
        super.onPause();
        mSMan.unregisterListener(this);
    }



    public void onSensorChanged(SensorEvent event) {
        int etype = event.sensor.getType();
        etime = event.timestamp;
        float dt=0;


        //Recod the value and time
        switch (etype) {
            case Sensor.TYPE_ACCELEROMETER:
                dAcc[0]=event.values[0]-cBAcc[0];
                dAcc[1]=event.values[1]-cBAcc[1];
                dAcc[2]=event.values[2]-cBAcc[2];
                if (ptAcc!=0) dt=(etime-ptAcc)/N2S;
                ptAcc=etime;
                break;
            case Sensor.TYPE_GYROSCOPE:
                dGyro[0]=event.values[0]-cBGyro[0];
                dGyro[1]=event.values[1]-cBGyro[1];
                dGyro[2]=event.values[2]-cBGyro[2];
                if (ptGyro!=0) dt=(etime-ptGyro)/N2S;
                ptGyro=etime;
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                dMag[0]=event.values[0];dMag[1]=event.values[1];dMag[2]=event.values[2];
                if (ptMag!=0) dt=(etime-ptMag)/N2S;
                ptMag=etime;

                break;
        }

        if (mode==0) {
            //do nothing
        }
        else if (mode==3) {	//6Dof Calculations
            if (etype==Sensor.TYPE_ACCELEROMETER) { //Update velocity and Pos
                //Update pos and vel
                mINS.update_velI(dAcc, dt);
                mINS.update_posII(dt);

                //Update acc accum
                mINS.accum.addacc(dAcc);
            }

            if (etype==Sensor.TYPE_GYROSCOPE) { //Update velocity and Pos
                //Update pos and vel
                mINS.update_attI(dGyro, dt);
                mINS.update_velII(dGyro, dt);
                mINS.update_posI(dGyro, dt);

                //Update acc accum
                mINS.accum.addgyro(dGyro);
            }



            //State Updates and Covariance propagation
            //当到了更新周期则更新视图
            //if (etime>tProp || ZFlag || CFlag) {
            if (true) {
                //First update (propagate the covariance to the current time)
                dt=(etime-ptProp)/N2S;
                ptProp=etime;

                //Propagate the covariance
                mKalman.Propagate(mINS, dt);

                //Clear sensor data accumulators
                mINS.accum.clear();

                //Next Covaraince update time
                tProp=etime+PERPROP;


                storePos.add(new float[]{(float) mINS.Pos_b.data[0],(float) mINS.Pos_b.data[1],(float) mINS.Pos_b.data[2]});

                //Debug screen
                etv.setText("Pos :" + mINS.Pos_b.data[0] + "\n" + mINS.Pos_b.data[1] + "\n"+ mINS.Pos_b.data[2] + "\n" +
                        "Vel :" + mINS.Vel_b.data[0] + "\n" + mINS.Vel_b.data[1] + "\n"+ mINS.Vel_b.data[2]);
                //etv.setText("Abias :" + cBAcc[0] + "\n" + cBAcc[1] + "\n"+ cBAcc[2] + "\n" +
                //	    "Gbias :" + cBGyro[0] + "\n" + cBGyro[1] + "\n"+ cBGyro[2]);

                //flow_control(4);

                //Check if there is an update request
                if (ZFlag) { //Process Zupt request
                    mKalman.applyZupt(mINS, cBAcc, cBGyro);
                    ZFlag=false;
                }

                if (CFlag) { //Process Cupt Request
                    mKalman.applyCupt(mINS, cBAcc, cBGyro, INIPos);
                    CFlag=false;
                }

            }


        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }


}
