package com.github.kke.inertia;

import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.Vec2;

public class v_GetCached_v extends NodeVec2 {

    private CachedValue embedded = null;

    public v_GetCached_v(CachedValue cv) {
        embedded = cv;
    }    

    @Override
    public Vec2 Value(long timestamp) {
        return embedded.getCached();
	}

}