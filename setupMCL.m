function mcl = setupMCL(odyModel,rangeModel,initPos,map)
    %METHOD1 Summary of this method goes here
    %   Detailed explanation goes here
    mcl = monteCarloLocalization;
    mcl.SensorModel.Map = map;
    
    mcl.UseLidarScan = true;
    mcl.MotionModel = odyModel;
    mcl.SensorModel = rangeModel;
    
    mcl.UpdateThresholds = [0.0, 0.0, 0.0];
    mcl.ResamplingInterval = 3;
    %mcl.ResamplingInterval = 1;
    
    mcl.ParticleLimits = [500 5000];
    %mcl.ParticleLimits = [500 2000];
    mcl.InitialPose = initPos;
    mcl.InitialCovariance = eye(3)*0.5;
end