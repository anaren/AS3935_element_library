    float value;
    
    /**
    Detect oncoming lightning storm fronts up to 40km away.
    @param id device ID (0 to 7) on i2c bus
    @return Estimated distance of oncoming storm front in km.
    */
    value = AS3935_GetDistanceEstimation(0);
    
    return value; 
