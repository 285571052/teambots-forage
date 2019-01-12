package com.github.kke.utils;

import EDU.gatech.cc.is.clay.NodeBoolean;
import EDU.gatech.cc.is.clay.NodeVec2Array;
import EDU.gatech.cc.is.util.Vec2;

public class b_NonZero_va extends NodeBoolean {

    NodeVec2Array embedded1 = null;
    boolean	last_val = false;
    long	lasttime = 0;

    public b_NonZero_va(NodeVec2Array array) {
        super();
        embedded1 = array;
    }

    /**
     Return a boolean indicating if the embedded schema
     is a non-zero length array.
     @param timestamp long, only get new information if timestamp > than last call
     or timestamp == -1.
     @return true if non-zero, false if zero.
     */
    public boolean Value(long timestamp)
    {
        if (DEBUG) System.out.println("b_NonZero_va: Value()");

        if ((timestamp > lasttime)||(timestamp == -1))
        {
            /*--- reset the timestamp ---*/
            if (timestamp > 0) lasttime = timestamp;

            /*--- compute the output ---*/
            Vec2[] tmp = embedded1.Value(timestamp);
            last_val = (tmp.length != 0);
        }

        return (last_val);
    }
}
