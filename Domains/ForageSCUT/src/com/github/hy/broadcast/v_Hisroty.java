
/*
 * v_Closest_va.java
 */
package com.github.hy.broadcast;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.clay.*;

public class v_Hisroty extends NodeVec2 {
    /**
     * Turns debugging on or off.
     */
    public static final boolean DEBUG = false;
    private v_Cache broadcast_cache, closest_cache;
    private static double aging_rate = 0.995;
    private long last_update = -1;

    /**
     * Instantiate a v_Hisroty node.
     *
     * @param im1 NodeVec2, the embedded node that generates a list of items to
     *            scan.
     */
    public v_Hisroty(v_Cache broadcast_cache, v_Cache closest_cache) {
        if (DEBUG)
            System.out.println("v_Hisroty: instantiated.");
        this.broadcast_cache = broadcast_cache;
        this.closest_cache = closest_cache;
        last_node = closest_cache;
        last_val.setr(0);
    }

    double last_r = 0;
    v_Cache last_node;
    Vec2 last_val = new Vec2();
    Vec2 zero_val = new Vec2(0, 0);

    long lasttime = 0;

    /**
     * Return a Vec2 representing the closest object, or 0,0 if none are visible.
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the vector.
     */
    public Vec2 Value(long timestamp) {
        if ((timestamp > lasttime) || (timestamp == -1)) {
            /*--- reset the timestamp ---*/
            if (timestamp > 0) {
                lasttime = timestamp;
            }

            closest_cache.Value(timestamp);
            broadcast_cache.Value(timestamp);

            if (closest_cache.CheckUpdateAfter(last_update)) {
                last_r = 1;
                last_node = closest_cache;
                last_update = closest_cache.last_update;
            } else if (broadcast_cache.CheckUpdateAfter(last_update)) {
                last_r = 1;
                last_node = broadcast_cache;
                last_update = broadcast_cache.last_update;
            }
        }

        last_val = last_node.Value(timestamp);
        if (last_r == 0 || last_val.r < 1) {
            last_r = 0;
            return (Vec2) zero_val.clone();
        }
        if (last_r > 0.05) {
            // System.out.println(timestamp + ":" + last_r);
            last_r = last_r * aging_rate;
        } else if (last_r > 0) {
            // System.out.println(timestamp + ":" + last_r);
            last_r = 0;
        }
        return (new Vec2(last_val.x, last_val.y));
    }
}
