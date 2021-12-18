function Trigger()
    proximitySensorDistance = sim.readProximitySensor(proximityTrigger) 
    if proximitySensorDistance>0 then
        Init=true
    end
end

function SelectDrop()
    local image=sim.getVisionSensorCharImage(visionSensor)
    if image:byte(1) > image:byte(2)and image:byte(1) > image:byte(3) then
        return 1 --rosso camshaft
    elseif image:byte(2) > image:byte(1) and image:byte(2) > image:byte(3) then
        return 2 --verde oiltray
    elseif image:byte(3) > image:byte(1) and image:byte(3) > image:byte(2) then
        return 3   --blu    oilpump  
    end
end

function IK_actuation()
    if simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_undamped,true)==simIK.result_fail then 
        simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_damped)
        if not ikFailedReportHandle then 
        end
    else
        if ikFailedReportHandle then
            sim.endDialog(ikFailedReportHandle) 
            ikFailedReportHandle=nil
        end
    end
end

function IK_init()

    simBase=sim.getObjectHandle(sim.handle_self)
    simTip=sim.getObjectHandle('RPP_Tip')
    simTarget=sim.getObjectHandle('RPP_Target')
    ikEnv=simIK.createEnvironment()
    ikGroup_undamped=simIK.createIkGroup(ikEnv)
    simIK.setIkGroupCalculation(ikEnv,ikGroup_undamped,simIK.method_pseudo_inverse,0,15)
    simIK.addIkElementFromScene(ikEnv,ikGroup_undamped,simBase,simTip,simTarget,simIK.constraint_pose)
    ikGroup_damped=simIK.createIkGroup(ikEnv)
    simIK.setIkGroupCalculation(ikEnv,ikGroup_damped,simIK.method_damped_least_squares,1,99)
    simIK.addIkElementFromScene(ikEnv,ikGroup_damped,simBase,simTip,simTarget,simIK.constraint_pose)
end
------------------------------------------------------------------------------------------
function extractTargetObjectsInScene(selectdrop)
    if selectdrop == 1 then
        
            handle = sim.getObjectHandle('CamShaft'..value1 .."@silentError") --@silentError will not display any error if object handle is missing
            if (handle ~= -1) then
                table.insert(targetHandles0, handle)
            end
            value1 = value1 + 1;
        
    elseif selectdrop == 2 then

             handle = sim.getObjectHandle('OilTray'..value2 .."@silentError") --@silentError will not display any error if object handle is missing
            if (handle ~= -1) then
                table.insert(targetHandles0, handle)
            end
            value2 = value2 + 1;

    elseif selectdrop == 3 then
           handle = sim.getObjectHandle('FuelPump'..value3 .."@silentError") --@silentError will not display any error if object handle is missing
            if (handle ~= -1) then
                table.insert(targetHandles0, handle)
            end
            value3 = value3 + 1;
        
    end
    return targetHandles0
end
function updateTargetPath(nextTargetObject, i)
--print ("updateTargetPath")
    currRobotPose = sim.getObjectPose(simTarget, -1);
    currTargetObject = nextTargetObject
    if i == 2 then
        currInteractionPoint = sim.getObjectChild(currTargetObject, 0) 
    elseif i == 1 or i == 3 then
        currInteractionPoint = sim.getObjectChild(currTargetObject, 1)
    else 
    end
    targetPose = sim.getObjectPose(currInteractionPoint, -1);
    return {currRobotPose, targetPose}  
end
function initAndExtractPathInfo(path)
    ctrlPts1 = {}
    for i=1,#path,1 do
      for j=1, #path[i], 1 do
         table.insert(ctrlPts1, path[i][j])
      end
    end
    local pathHandle0 = sim.createPath(ctrlPts1,8,100,1.0,0,{0,0,1})
    pathData=sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle0,'PATH'))
    local m=Matrix(#pathData//7,7,pathData)
    pathPositions=m:slice(1,1,m:rows(),3):data()
    pathQuaternions=m:slice(1,4,m:rows(),7):data()
    pathLengths,totalLength=sim.getPathLengths(pathPositions,3)
    velocity=0.3 -- m/s
    posAlongPath=0
    stopPosition = {pathPositions[#pathPositions-2],pathPositions[#pathPositions-1], pathPositions[#pathPositions]};
    isTargetReached = false;
    return pathHandle0
end

function extractDropObjectsInScene()
    local dropPointHandles0={}
    local dropHandle = 0;
    local j = 1;
    while dropHandle ~= -1 do
        dropHandle = sim.getObjectHandle('Drop'..j .."@silentError") --@silentError will not display any error if object handle is missing
        if (dropHandle ~= -1) then
            table.insert(dropPointHandles0, dropHandle)
        end
        j = j + 1;
    end
    return dropPointHandles0
end


---------------------------------------------------------------------
function followInterpolatedPath(pathHandle0, isLoop)
    if (isTargetReached == false) then
        local dist = 0;
        local t=sim.getSimulationTime()
        posAlongPath=posAlongPath+velocity*(t-previousSimulationTime)
        if isLoop then
            posAlongPath=posAlongPath % totalLength
            isTargetReached = false;
        end
        local pos=sim.getPathInterpolatedConfig(pathPositions,pathLengths,posAlongPath)
        local quat=sim.getPathInterpolatedConfig(pathQuaternions,pathLengths,posAlongPath,nil,{2,2,2,2})
        sim.setObjectPosition(simTarget,pathHandle0,pos)
        sim.setObjectQuaternion(simTarget,pathHandle0,quat)
        previousSimulationTime=t
    end
end
function updateDropPointPath(nextDropPoint)
    currRobotPose = sim.getObjectPose(simTarget, -1);
    local offsetRobotPose = currRobotPose;
    offsetRobotPose[3] = offsetRobotPose[3] +0.1;
    currInteractionPoint = nextDropPoint
    targetPose = sim.getObjectPose(currInteractionPoint, -1);
    return {currRobotPose,offsetRobotPose, targetPose}  
end
-----------------------------------------------------------
function sysCall_init()
    flag =false
    targetHandles0={}
    handle = 0;
    handlato = false
    onetime = true
    Init = false
    value1 = 0;
    value2 = 0;
    value3 = 0;
    proximityTrigger = sim.getObjectHandle('Proximity_sensor0')
    visionSensor=sim.getObjectHandle('Vision_sensor')
    IK_init()
    currTargetIdx = 0;
    previousSimulationTime=0;
    cach = false;
    isAllTargetComplete = false;
    distanceSegment=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})
    isDropPointSet = false;
    isDropPointReached = false;
    suctionPad = sim.getObjectHandle('suctionPad')
    waitTimer = 0;    
    dropPointHandles = extractDropObjectsInScene();
    noOfDropPoints = #dropPointHandles;
    wait = true
end

function sysCall_actuation()
    IK_actuation()
    if Init == true then
        if onetime == true then
            print ("dentro onetime")
            currTargetIdx = currTargetIdx + 1
            selectdrop = SelectDrop()
            targetHandles = extractTargetObjectsInScene(selectdrop);--sono sicuro sia giusto
            noOfTargets = #targetHandles;
            handlato = true
            path = updateTargetPath(targetHandles[currTargetIdx], selectdrop)
            pathHandle = initAndExtractPathInfo(path)
            wait = false
            onetime = false
        end
        
        if wait == false then 
            followInterpolatedPath(pathHandle, false)
            print ("calcolo la path")
            
        end
        if (isTargetReached and isDropPointReached  and isDropPointSet== false ) then
            if currTargetIdx ~= noOfTargets then 
                currTargetIdx = currTargetIdx + 1; 
            end
            if (currTargetIdx <= noOfTargets and isAllTargetComplete == false) then
                path = updateTargetPath(targetHandles[currTargetIdx])
                pathHandle = initAndExtractPathInfo(path)
                isDropPointReached=false
                isTargetReached =false           
            end
        end
        anchorPoint = sim.getObjectChild(targetHandles[currTargetIdx],1)
        if (anchorPoint ~=-1  and isDropPointSet==false and isDropPointReached==false and flag == true) then
            print ("cerca la path del drop ")
            wait = true
            isTargetReached = false;
            currDropPointIdx = selectdrop
            path = updateDropPointPath(dropPointHandles[currDropPointIdx]) --qui
            pathHandle = initAndExtractPathInfo(path)
            isDropPointSet = true;
            wait = false
        end
        if (isDropPointSet == true and isTargetReached == true ) then
            print("Drop Point Reached")
            BlockDrop = false
            if (isDropPointReached == false) then
                waitTimer = sim.getSimulationTime();
            end
            isDropPointReached = true
            
            sim.setLinkDummy(sim.getObjectHandle('suctionPadLoopClosureDummy1'),-1)
            local timeExpired = sim.getSimulationTime()-waitTimer;
            if (timeExpired > 5) then
                sim.setObjectParent(sim.getObjectHandle('suctionPadLoopClosureDummy1'),suctionPad,false)-- reparent to the suctionPad
                isDropPointSet = false;
                isTargetReached = false;
                onetime = true
                flag = false
            end
        end  
    end 
end 


function sysCall_sensing()
        Trigger()
       
        if Init == true then
            if handlato == true then 
                local result,distData,objectPair=sim.checkDistance(simTip,currInteractionPoint)
                if result>0 then
                    if (distData[7] < 0.03) then
                      isTargetReached = true;
                      flag = true
                    print (distData[7])
                      --onetime = true
                      --print(--print (distData[7]))
                     --  --print("Is Targed Reached: " .. tostring(isTargetReached) .."\tObject Reached: " .. sim.getObjectName(currInteractionPoint))
                    end
                    sim.addDrawingObjectItem(distanceSegment,nil)
                    sim.addDrawingObjectItem(distanceSegment,distData)
                end
            end
    end
  
end
function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv) 
end 