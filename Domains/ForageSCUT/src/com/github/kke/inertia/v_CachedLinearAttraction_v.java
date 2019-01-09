package com.github.kke.inertia;


import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.*;

public class v_CachedLinearAttraction_v extends v_LinearAttraction_v implements CachedValue  {
    private Vec2 cached = null;
    /**
	Instantiate a v_LinearAttraction_v schema.
	@param czr double, controlled zone radius.
	@param dzr double, dead zone radius.
	@param im1 double, the node that generates an
		egocentric vector to the goal.
	*/
	public v_CachedLinearAttraction_v(double czr, double dzr, NodeVec2 im1) {
        super(czr, dzr, im1);
    }

    /**
	Return a Vec2 representing the direction to go towards the
	goal.  Magnitude varies with distance.
	@param timestamp long, only get new information if timestamp > than last call
                or timestamp == -1.
	@return the movement vector.
    */
    @Override
	public Vec2 Value(long timestamp) {
        cached = super.Value(timestamp);
        setCache(cached);
        return cached;
    }

    @Override
    public Vec2 getCached() {
        return cached;
    }

    @Override
    public void setCache(Vec2 v) {
        cached = new Vec2(v.x, v.y);
    }
}

/*
git checkout self
git merge other
*/