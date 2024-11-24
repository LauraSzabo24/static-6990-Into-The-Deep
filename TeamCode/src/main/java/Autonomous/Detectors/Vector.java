package Autonomous.Detectors;

import java.util.ArrayList;

public class Vector {

    public double magnitude;
    public ArrayList<Double> values;
    public Vector(ArrayList<Double> vals)
    {
        values = vals;
        double sumOfSquares = 0;
        for(int i=0; i<vals.size(); i++)
        {
            sumOfSquares+= vals.get(i)* vals.get(i);
        }
        magnitude = Math.sqrt(sumOfSquares);
    }

    public double getMagnitude() {
        return magnitude;
    }
    public ArrayList<Double> getValues() {
        return values;
    }
    public double i()
    {
        if(values.size()>0) {
            return values.get(0);
        }
        return 0;
    }
    public double j()
    {
        if(values.size()>1) {
            return values.get(1);
        }
        return 0;
    }
    public double k()
    {
        if(values.size()>2) {
            return values.get(2);
        }
        return 0;
    }
}
