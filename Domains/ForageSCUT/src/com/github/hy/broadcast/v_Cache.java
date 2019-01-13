
/*
 * v_Cache.java
 * Cache a local position as global position.
 */
package com.github.hy.broadcast;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.clay.*;

public class v_Cache extends NodeVec2 {
    /**
     * Turns debugging on or off.
     */
    public static final boolean DEBUG = false;
    private NodeVec2 pos_local, robot_pos_global;
    NodeBoolean cache_triger;
    private Vec2 last_val = new Vec2();
    public long last_update;

    /**
     * Instantiate a v_Cache node.
     *
     * @param pos_local        NodeVec2, the position in local mode.
     * @param robot_pos_global NodeVec2. the position of the robot in global mode.
     *
     */
    public v_Cache(NodeVec2 pos_local, NodeVec2 robot_pos_global, NodeBoolean cache_triger) {
        if (DEBUG)
            System.out.println("v_Cache: instantiated.");
        last_val.setr(0);
        this.pos_local = pos_local;
        this.robot_pos_global = robot_pos_global;
        this.cache_triger = cache_triger;
        last_update = -1;
    }

    long lasttime = 0;

    /**
     * Recompute and return the local position when call the Value function
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
            Vec2 pos_local_val = pos_local.Value(timestamp);
            Vec2 robot_pos_global_val = robot_pos_global.Value(timestamp);
            boolean cache_triger_val = cache_triger.Value(timestamp);
            // Cache the pos only when the pos_local_val is valid.

            if (cache_triger_val) {
                pos_local_val.add(robot_pos_global_val);
                last_val = (Vec2) pos_local_val.clone();
                last_update = timestamp;
            }
        }

        Vec2 ret_val = (Vec2) last_val.clone();
        Vec2 robot_pos_global_val = robot_pos_global.Value(timestamp);
        if (last_update == -1)
            return (new Vec2(0, 0));
        ret_val.sub(robot_pos_global_val);
        return (new Vec2(ret_val.x, ret_val.y));
    }

    public void ClearCache() {
        last_val.setr(0);
    }

    public boolean CheckUpdateAfter(long timestamp) {
        return last_update > timestamp;
    }
}
