function ConveyorControl()
    proximitySensorDistance = sim.readProximitySensor(sensorHandle)
       
     
    if proximitySensorDistance>0 then
        sim.writeCustomDataBlock(conveyorHandle,'CONVMOV',sim.packTable({vel=0}))

    else
        sim.writeCustomDataBlock(conveyorHandle,'CONVMOV',sim.packTable({vel=-0.1}))
    end
end

function Max2Iterations()
    while (flag == false) do
        if rando == 1 then
            if contatore1<2 then
            contatore1 = contatore1 + 1;
            contatore2 = 0;
            contatore3 = 0;
            flag = true
            else
            rando = math.random(2, 3)
            flag = false;
            end
        elseif rando == 2 then
            if contatore2<2 then
            contatore2 = contatore2 + 1;
            contatore1 = 0;
            contatore3 = 0;
            flag = true;
            else
            rando = math.random(1, 3)
            flag = false;
            end
        else 
            if contatore3<2 then
            contatore3 = contatore3 + 1;
            contatore1 = 0;
            contatore2 = 0;
            flag = true;
            
            else
            rando = math.random(1, 2)
            flag = false;
            end
        end
    end
end
function SpawnParts()
    spawnZoneEmpty = sim.readProximitySensor(spawnerSensorHandle);
    if((sim.getSimulationTime()-spawnTimer) >= 3*spawnInterval and spawnZoneEmpty==0) then
        initialSelection=sim.getObjectSelection();
        sim.removeObjectFromSelection(sim.handle_all,0);
        rando = math.random(1, 3)

        
        
        Max2Iterations();
        flag = false;
        
        ----------------------------------------------
        
        --sim.addObjectToSelection(sim.handle_single,CvxHandle[rando]);
        --sim.addObjectToSelection(sim.handle_single,meshHandle[rando]);
        --sim.addObjectToSelection(sim.handle_single,Trgt[rando]);
        sim.addObjectToSelection(sim.handle_tree,CvxHandle[rando]);
                
        copiedObj=simCopyPasteSelectedObjects();
        if rando == 1 then
            CopyHandle = sim.getObjectHandle('CamShaft' .. noOfObjectSpawned[rando]);
        end
        if rando == 2 then
            CopyHandle = sim.getObjectHandle('FuelPump' .. noOfObjectSpawned[rando]);
        end
        if rando == 3 then
            CopyHandle = sim.getObjectHandle('OilTray' .. noOfObjectSpawned[rando]);
        end
        print (CopyHandle)
        sim.setObjectPosition(CopyHandle,spawnPointHandle,{0,0,0});
        sim.removeObjectFromSelection(sim.handle_all,0);
        sim.addObjectToSelection(initialSelection);
        noOfObjectSpawned[rando] = noOfObjectSpawned[rando] + 1;
        spawnTimer = sim.getSimulationTime();
    end
end


function sysCall_init()
    sensorHandle = sim.getObjectHandle(sim.handle_self)
    conveyorHandle=sim.getObjectHandle('conveyor_back')
    spawnPointHandle = sim.getObjectHandle('SpawnPoint')
    spawnerSensorHandle = sim.getObjectHandle('Sensor_Spawn')
    
    --sim.writeCustomDataBlock(conveyorHandle,'CONVMOV',sim.packTable({vel=-0.1}))
    
    spawnInterval = 3; 
    spawnTimer = sim.getSimulationTime();
    noOfObjectSpawned = {0,0,0};
    
    -- Getting Object Handles
    meshHandle = {}
    CvxHandle = {}
    Trgt = {}
    --posizione = {0,0,0};
    meshHandle[1] = sim.getObjectHandle('CamShaft_shape')
    meshHandle[2] = sim.getObjectHandle('FuelPump_shape')
    meshHandle[3] = sim.getObjectHandle('OilTray_shape')
    
    ----------------------------------------------------
    Trgt[1] = sim.getObjectHandle('InteractionPointCamShaft')
    Trgt[2] = sim.getObjectHandle('InteractionPointFuelPump')
    Trgt[3] = sim.getObjectHandle('InteractionPointOilTray')
    -----------------------------------------------------
    CvxHandle[1] = sim.getObjectHandle('CamShaft')
    CvxHandle[2] = sim.getObjectHandle('FuelPump')
    CvxHandle[3] = sim.getObjectHandle('OilTray')
    flag = false;
    contatore1 = 0;
    contatore2 = 0;
    contatore3 = 0;
end

function sysCall_actuation()
    ConveyorControl();    
    SpawnParts();
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

