import os
import sys
import InitSetUp 
import OutputManager

import datetime
import timeit
import time

from Driver import Driver
import EvolutionaryLearner


# Importing needed python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci

if __name__ == "__main__":

    # --- TRAINING OPTIONS ---
    gui = False
    totalGenerations = 50
    individualRunsPerGen = 3  # Min number of training runs an individual gets per generation
    gamma = 0.75
    batch_size = 100
    memory_size = 50000
    path = "./model/model_1_5x400_100e_075g/"  # nn = 5x400, episodes = 300, gamma = 0.75
    # ----------------------

    # Attributes of the simulation
    sumoNetworkName = "simpleNetwork.net.xml"
    maxGreenPhaseTime = 225
    maxYellowPhaseTime = 5
    maxSimulationTime = 10000
    runTimeSet = []


    # setting the cmd mode or the visual mode
    if gui == False:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # initializations
    #sumoCmd = [sumoBinary, "-c", "intersection/tlcs_config_train.sumocfg", "--no-step-log", "true", "--waiting-time-memory", str(max_steps)]
    sumoCmd = [sumoBinary, "-c", "config_file.sumocfg", "--waiting-time-memory", "5", "--time-to-teleport", "-1"]
        
    print("----- Start time:", datetime.datetime.now())
    setUpTuple = InitSetUp.run(sumoNetworkName, individualRunsPerGen)
    simRunner = Driver(sumoCmd, setUpTuple, maxGreenPhaseTime, maxYellowPhaseTime, maxSimulationTime)
    episode = 0
    generations = 1
    allIndividualsTested = False
    simulationStartTime = datetime.datetime.now()
    generationRuntimes = []

    # Evolutionary learning loop 
    while generations <= totalGenerations:
        print('----- GENERATION {} of {}'.format(generations, totalGenerations))
        print("This simulation began at:", simulationStartTime)
        print("The average generation runtime is", sum(generationRuntimes)/generations)
        genStart = datetime.datetime.now()
        startTime = time.time()

        # Prepare for next simulation run
        allIndividualsTested = False
        for ap in setUpTuple[2]:
            for i in ap.getIndividualsSet():
                i.resetSelectedCount()
                # print("Generation includes Individual:", i.getID())

        # Reinforcement learning loop
        while not allIndividualsTested:
                # Adjust maximum simulation times for individuals based on generation count
            if generations >= 5 and generations < 15:
                print('The generation is', generations, "so we're changing maxSimTime to 6000")
                maxSimulationTime = 6000
                print("Changed maxSimTime to", maxSimulationTime)
            elif generations >= 15:
                print('The generation is', generations, "so we're changing maxSimTime to 4000")
                maxSimulationTime = 4000
                print("Changed maxSimTime to", maxSimulationTime)

            print('Changes made. The generation is', generations, "and the maxSimTime is", maxSimulationTime)
            simRunner = Driver(sumoCmd, setUpTuple, maxGreenPhaseTime, maxYellowPhaseTime, maxSimulationTime)

            print('----- Episode {}'.format(episode+1), "of GENERATION {} of {}".format(generations, totalGenerations))
            print("Generation start time:", genStart)
            print("The average generation runtime is", sum(generationRuntimes)/generations)
            start = timeit.default_timer()
            resultingAgentPools = simRunner.run()  # run the simulation
            stop = timeit.default_timer()
            print('Time: ', round(stop - start, 1))
            episode += 1

            needsTesting = []
            for ap in resultingAgentPools:
                for i in ap.getIndividualsSet():
                    if i.getSelectedCount() < individualRunsPerGen:
                        needsTesting.append(True)
                    else:
                        needsTesting.append(False)
            
            if True not in needsTesting:
                allIndividualsTested = True
                for ap in resultingAgentPools:
                    for i in ap.getIndividualsSet():
                        continue # print(i, "has a selected count of:", i.getSelectedCount())
            #allIndividualsTested = True # Uncomment for quick testing

            # Prepare individuals for the next run through
        for ap in setUpTuple[2]:
            ap.normalizeIndividualsFitnesses()  # Normalize the fitness values of each Individual in an agent pool for breeding purposes
                
        if generations + 1 < totalGenerations:
            EvolutionaryLearner.createNewGeneration(setUpTuple[2])     # Update agent pools with a new generation of individuals
            for ap in setUpTuple[2]:
                for i in ap.getIndividualsSet():
                    i.resetSelectedCount()
                    i.resetAggregateVehicleWaitTime()
                    # print("Generation includes Individual:", i.getID(), ";\n")
            sys.stdout.flush()
        else:
            OutputManager.run(setUpTuple[2], sum(generationRuntimes)/50, (sum(generationRuntimes)/50)*50)
            print("Output file created.")
        
        #     bestIndividuals = []
        #     for ap in setUpTuple[2]:
        #         bestIndividuals.append(ap.getBestIndividual())
            
        #     f = open("bestIndividuals.txt", "w")
            
        #     for i in bestIndividuals:
        #         f.write("The best individual in Agent Pool", i.getAgentPool().getID(), "is", i.getID(), "comprised of conditions:", i.getConditions(), "and action:", i.getAction(), "\n\n")
        
        print("Generation start time:", genStart, "----- End time:", datetime.datetime.now())
        generationRuntimes.append(time.time() - startTime)
        generations += 1 
               

    print("Start time:", simulationStartTime, "----- End time:", datetime.datetime.now())
    print("This simulation began at:", simulationStartTime)
    print("PATH:", path)
    # Do something to save session stats here