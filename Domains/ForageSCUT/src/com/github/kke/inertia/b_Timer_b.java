package com.github.kke.inertia;

import EDU.gatech.cc.is.clay.*;

class b_Timer_b extends NodeBoolean {

    private long expired_count = 0;
    private long maximum_expiration = 1;
    
    private NodeBoolean embedded = null;
    private boolean	last_val     = false;
    private long lasttime        = 0;

    public b_Timer_b(NodeBoolean node, long expiration) {
        super();
        embedded = node;
        maximum_expiration = expiration;
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

            boolean reset_timer = embedded.Value(timestamp);
            if (reset_timer) {
                expired_count = 0; 
                last_val = false;
            } else if (++expired_count > maximum_expiration) {
                // timeout
                expired_count = 0;
                last_val = true;
            } else {
                last_val = false;
            }
        }
        
    return (last_val);
    
    }

}