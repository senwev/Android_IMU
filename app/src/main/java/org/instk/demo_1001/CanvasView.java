package org.instk.demo_1001;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.View;

import java.util.List;

public class CanvasView extends View {


    List<float[]> dataList ;
    public CanvasView(Context context) {
        super(context);
    }

    public CanvasView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
    }

    public CanvasView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
    }

    public CanvasView(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
    }

    public void setDataList(List<float[]> dataList) {
        this.dataList = dataList;
        postInvalidate();
    }

    @Override
    public void draw(Canvas canvas) {
        super.draw(canvas);

        if(dataList!=null)
        {

            Paint paint = new Paint();
            paint.setColor(Color.RED);

            for(float[] a : dataList)
            {
                canvas.drawPoint(500+a[0]*100,500+a[1]*100,paint);
            }

        }



    }
}
