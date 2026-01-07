function sysCall_init()
obstacleThreshold = 0.9   -- meters

    sim = require('sim')
    simROS2 = require('simROS2')
    
    -- ===== OBJECT HANDLES =====
    lidar = sim.getObject("/lidar")
    
    -- Joints
    joint_left       = sim.getObject("/joint_left")
    joint_right      = sim.getObject("/joint_right")
    joint_front_left = sim.getObject("/front_left")
    joint_front_right= sim.getObject("/front_right")
    
    -- Proximity sensor
    proximitySensor = sim.getObject("/proximitySensor")
    
    -- ===== LIDAR CONFIGURATION =====
    sim.setObjectFloatParam(lidar, sim.visionfloatparam_near_clipping, 0.1)
    sim.setObjectFloatParam(lidar, sim.visionfloatparam_far_clipping, 10.0)
    
    local fov = math.rad(270)
    sim.setObjectFloatParam(lidar, sim.visionfloatparam_perspective_angle, fov)
    
    -- ===== ENABLE VELOCITY CONTROL =====
    sim.setObjectInt32Param(joint_left,        2000, 1)
    sim.setObjectInt32Param(joint_right,       2000, 1)
    sim.setObjectInt32Param(joint_front_left,  2000, 1)
    sim.setObjectInt32Param(joint_front_right, 2000, 1)
    
    -- ===== MOTOR FORCE =====
    sim.setJointMaxForce(joint_left,        30)
    sim.setJointMaxForce(joint_right,       30)
    sim.setJointMaxForce(joint_front_left,  30)
    sim.setJointMaxForce(joint_front_right, 30)
    
    -- ===== ROS 2 SETUP =====
    if simROS2 then
        laserScanPub = simROS2.createPublisher('/scan', 'sensor_msgs/msg/LaserScan')
        cmdVelSub = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'cmdVelCallback')
        print("ROS 2 initialized")
    else
        print("ERROR: simROS2 module not found!")
    end
    
    -- ===== ROBOT STATE =====
    currentLinearVel = 0.0
    currentAngularVel = 0.0
    
    wheelRadius = 0.05
    wheelBase = 0.3

    -- ===== OBSTACLE AVOIDANCE STATE (FROM CONCEPT SCRIPT) =====
    backUntilTime = -1
    avoidanceActive = false
    avoidSpeed = 8.0   -- rad/s (safe slow speed)

    print("Robot initialized successfully")
end

-- ===== TELEOP CALLBACK =====
function cmdVelCallback(msg)
    currentLinearVel = msg.linear.x
    currentAngularVel = msg.angular.z

    -- Only apply teleop if NOT avoiding obstacle
    if not avoidanceActive then
        local leftVel, rightVel = twistToDiffDrive(currentLinearVel, currentAngularVel)
        setAllWheels(leftVel, rightVel)
    end
end

function twistToDiffDrive(linear, angular)
    local leftVel  = (linear + angular * wheelBase / 2) / wheelRadius
    local rightVel = (linear - angular * wheelBase / 2) / wheelRadius
    return leftVel, rightVel
end

function setAllWheels(leftSpeed, rightSpeed)
    sim.setJointTargetVelocity(joint_left,        leftSpeed)
    sim.setJointTargetVelocity(joint_front_left,  leftSpeed)
    sim.setJointTargetVelocity(joint_right,       rightSpeed)
    sim.setJointTargetVelocity(joint_front_right, rightSpeed)
end

function sysCall_actuation()
    local detected, distance =
        sim.readProximitySensor(proximitySensor)

    -- Print obstacle distance
    if detected > 0 then
        sim.addLog(sim.verbosity_scriptinfos,
            string.format("Obstacle distance: %.2f m", distance))
    end

    -- Trigger avoidance when too close
    if detected > 0 and distance < obstacleThreshold then
        backUntilTime = sim.getSimulationTime() + 0.3
        avoidanceActive = true
    end

    -- Avoidance behavior (same BubbleRob concept)
    if avoidanceActive then
        if sim.getSimulationTime() < backUntilTime then
            -- Back + turn
            setAllWheels(-avoidSpeed, avoidSpeed)
        else
            avoidanceActive = false
        end
    end
end

function sysCall_sensing()
    local result, data = sim.readVisionSensor(lidar)
    
    if result >= 0 and data and data[2] then
        local aux = data[2]
        local scanMsg = {}
        
        scanMsg.header = {
            stamp = simROS2.getTime(),
            frame_id = 'lidar'
        }
        
        scanMsg.angle_min = -2.356
        scanMsg.angle_max = 2.356
        scanMsg.range_min = 0.1
        scanMsg.range_max = 10.0
        
        if type(aux) == "table" then
            local numRays = #aux
            scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (numRays - 1)
            scanMsg.time_increment = 0.0
            scanMsg.scan_time = 0.1
            
            scanMsg.ranges = {}
            for i = 1, numRays do
                local d = aux[i]
                if d < scanMsg.range_min or d > scanMsg.range_max then
                    d = scanMsg.range_max
                end
                table.insert(scanMsg.ranges, d)
            end
            
            scanMsg.intensities = {}
            for i = 1, numRays do
                table.insert(scanMsg.intensities, 0.0)
            end
            
            if laserScanPub then
                simROS2.publish(laserScanPub, scanMsg)
            end
        end
    end
end

function sysCall_cleanup()
    if simROS2 then
        if laserScanPub then simROS2.shutdownPublisher(laserScanPub) end
        if cmdVelSub then simROS2.shutdownSubscription(cmdVelSub) end
    end
    
    setAllWheels(0, 0)
    print("Robot script cleaned up")
end

