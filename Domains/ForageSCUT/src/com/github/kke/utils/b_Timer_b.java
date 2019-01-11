package com.github.kke.utils;

import EDU.gatech.cc.is.clay.*;

public class b_Timer_b extends NodeBoolean {

    private long expired_count = 0;
    private long maximum_expiration = 1;
    
    private NodeBoolean embedded = null;
    private boolean	last_val     = false;
    private boolean auto_reset   = false;
    private long lasttime        = 0;

    public b_Timer_b(NodeBoolean node, long expiration, boolean auto) {
        super();
        embedded = node;
        maximum_expiration = expiration;
        auto_reset = auto;
    }

    /*
     * Reset timer when embedded.Value() == true
     * Return 1 on timeout
     */
    @Override
	public boolean Value(long timestamp)
    {
        // if (DEBUG) System.out.println("b_Timer_b: Value()");
        if ((timestamp > lasttime)||(timestamp == -1))
        {
            /*--- reset the timestamp ---*/
            if (timestamp > 0) lasttime = timestamp;
            //
            // acquire->inertia 的条件是 target not visible
            // reset_timer 肯定为 false，不能触发重置定时器
            // 如果 last_value 已经 expired, 定时器就不能重置了
            // 定时器只会在 inertia 状态下调用 Value()
            //
            boolean reset_timer = embedded.Value(timestamp);
            if (reset_timer) {
                expired_count = 0; 
                last_val = false;
            } else if (++expired_count > maximum_expiration) {
                // timeout
                if (auto_reset)
                    expired_count = 0;
                else
                    expired_count = maximum_expiration;
                last_val = true;
            } else {
                last_val = false;
            }
        }
        
    return (last_val);
    
    }

}