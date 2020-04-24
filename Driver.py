#!/usr/bin/env python

import os
import sys
import optparse
import traci

import PredicateSet 
import CoopPredicateSet
import EvolutionaryLearner 
import ReinforcementLearner
from Rule import Rule
from Intention import Intention

class Driver:

    global userDefinedRules
    global trafficLights
    global rule
    global nextRule
    global maxSimulationTime

    def __init__(self, sumoCmd, setUpTuple, maxGreenPhaseTime, maxYellowPhaseTime, maxSimulationTime):
        self.sumoCmd = sumoCmd
        self.setUpTuple = setUpTuple
        self.maxGreenPhaseTime = maxGreenPhaseTime
        self.maxYellowPhaseTime = maxYellowPhaseTime
        self.maxSimulationTime = maxSimulationTime


    # CONTAINS MAIN TRACI SIMULATION LOOP
    def run(self):
        traci.start(self.sumoCmd)   # Start SUMO. Comment out if running Driver as standalone module.

            # Run set-up script and acquire list of user defined rules and traffic light agents in simulation
        userDefinedRules = self.setUpTuple[0]
        trafficLights = self.setUpTuple[1]
        rule = None 
        nextRule = None

            # Assign each traffic light an individual from their agent pool for this simulation run, and a starting rule
        for tl in trafficLights:
            tl.assignIndividual()

            rule = self.applicableUserDefinedRule(tl, userDefinedRules) # Check user-defined rules
                
                # If no user-defined rules can be applied, get a rule from Agent Pool
            if rule == False:    
                validRules = self.getValidRules(tl, tl.getAssignedIndividual())
                rule = tl.getNextRule(validRules[0], validRules[1], traci.simulation.getTime()) # Get a rule from assigned Individual
                    
                    # if no valid rule applicable, apply the Do Nothing rule.
                if rule == -1:
                    # print("No valid rule. Do Nothing action applied.") 
                    tl.doNothing()  # Update traffic light's Do Nothing counter
                    tl.getAssignedIndividual().updateFitnessPenalty(False, 0)   # Update fitness penalty for individual

                else:       
                        # If rule conditions are satisfied, apply its action. Otherwise, do nothing.
                    if not rule.hasDoNothingAction():
                        traci.trafficlight.setPhase(tl.getName(), rule.getAction())                
            else:
                self.applyUserDefinedRuleAction(tl, traci.trafficlight.getPhaseName(tl.getName()), rule)

            tl.setCurrentRule(rule) # Set current rule in traffic light

            # Simulation loop 
        step = 0
            # Variables for rule rewards
        carsWaitingBefore = {}
        carsWaitingAfter = {}
        while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime() < self.maxSimulationTime:
            traci.simulationStep() # Advance SUMO simulation one step (1 second)

                # Traffic Light agents reevaluate their state every 5 seconds
            if step % 5 == 0:  
                    # For every traffic light in simulation, select and evaluate new rule from its agent pool
                for tl in trafficLights:
                    carsWaitingBefore = tl.getCarsWaiting()
                    carsWaitingAfter = self.carsWaiting(tl) 
                    
                    nextRule = self.applicableUserDefinedRule(tl, userDefinedRules) # Check if a user-defined rule can be applied
                    
                        # If no user-defined rules can be applied, get a rule from Agent Pool
                    if nextRule == False:    
                        validRules = self.getValidRules(tl, tl.getAssignedIndividual())
                        nextRule = tl.getNextRule(validRules[0], validRules[1], traci.simulation.getTime()) # Get a rule from assigned Individual

                            # if no valid rule applicable, apply the Do Nothing rule.
                        if nextRule == -1:
                            tl.doNothing()  # Update traffic light's Do Nothing counter
                            tl.getAssignedIndividual().updateFitnessPenalty(False, False)   # Update fitness penalty for individual
                            
                            # If next rule is not a user-defined rule, update the weight of the last applied rule
                        else:
                            oldRule = tl.getCurrentRule()
                                
                                # If applied rule isn't user-defined, update its weight
                            if oldRule not in userDefinedRules:
                                if oldRule != -1:
                                    ruleWeightBefore = oldRule.getWeight()   # Used to calculate fitness penalty to individual
                                    oldRule.updateWeight(ReinforcementLearner.updatedWeight(oldRule, nextRule, self.getThroughputRatio(self.getThroughput(tl, carsWaitingBefore, carsWaitingAfter), len(carsWaitingBefore)), self.getWaitTimeReducedRatio(self.getThroughputWaitingTime(tl, carsWaitingBefore, carsWaitingAfter), self.getTotalWaitingTime(carsWaitingBefore)), len(carsWaitingAfter) - len(carsWaitingBefore)))
                                    tl.getAssignedIndividual().updateFitnessPenalty(True, oldRule.getWeight() > ruleWeightBefore)
                                    print("Old weight was", ruleWeightBefore, "and new weight is", oldRule.getWeight())
                                    # Apply the next rule; if action is -1 then action is do nothing
                                if not nextRule.hasDoNothingAction():
                                    # print('Next rule action is', nextRule.getAction())
                                    traci.trafficlight.setPhase(tl.getName(), nextRule.getAction())
                                
                                if nextRule.getType() == 0:
                                    print("Applying TL action from RS! Action is", nextRule.getAction(), "\n\n")                
                                else:
                                    print("Applying TL action from RSint! Action is", nextRule.getAction(), "\n\n")                

                    else:
                        self.applyUserDefinedRuleAction(tl, traci.trafficlight.getPhaseName(tl.getName()), nextRule)
                        # # print("Applying action of", nextRule.getConditions())  

                    tl.setCurrentRule(nextRule)                 # Update the currently applied rule in the traffic light
                    tl.updateCarsWaiting(carsWaitingAfter)      # Set the number of cars waiting count within the TL itself

            step += 1  # Increment step in line with simulator
            
            # Update the fitnesses of the individuals involved in the simulation based on their fitnesses
        simRunTime = traci.simulation.getTime()
        print("***SIMULATION TIME:", simRunTime, "\n\n")
        for tl in trafficLights:
            # # print(tl.getName(), "has these communicated intentions:", tl.getCommunicatedIntentions())
            i = tl.getAssignedIndividual()
            i.updateLastRunTime(simRunTime)
            print("Individual", i, "has a last runtime of", i.getLastRunTime())
            i.updateFitness(EvolutionaryLearner.rFit(i, simRunTime, i.getAggregateVehicleWaitTime()))

        traci.close()       # End simulation
        
        return self.setUpTuple[2] # Returns all the agent pools to the main module
        # sys.stdout.flush()

        
    # RETRIEVE THE STATE OF THE INTERSECTION FROM SUMO
    def getState(self, trafficLight):
        state = {}
        leftTurnLane = ""
        for lane in trafficLight.getLanes():
            state[lane] = []

            # Loop to determine which vehicles are waiting at an intersection
        for vehID in traci.vehicle.getIDList(): 
            laneID = traci.vehicle.getLaneID(vehID)
            tlLanes = trafficLight.getLanes()
            
            # Operate only on vehicles in a lane controlled by traffic light 
            if laneID in tlLanes:
                    # Determine left turn lane if it exists
                if "_LTL" in laneID:
                    maxLaneNum = 0
                    for lane in tlLanes:
                        if lane == laneID:
                            laneSplit = lane.split("_")
                            if int(laneSplit[2]) > maxLaneNum:
                                leftTurnLane = lane
                
                    # If vehicle is stopped, append relevant identifier to it
                if traci.vehicle.getSpeed(vehID) == 0:    
                    if leftTurnLane == laneID:
                        vehID = vehID + "_L"
                    else:
                        vehID = vehID + "_S"
                
                    state[laneID].append(vehID)
                
        return state

        # RETURNS A DICTIONARY WITH KEYS OF VEHIDs WAITING AT AN INTERSECTION AND THEIR WAITING TIME AS VALUES
    def carsWaiting(self, trafficLight):
        state = self.getState(trafficLight)
        carsWaiting = {}
            # Count all vehicles in the state dictionary
        for lanes in state:
            for veh in state[lanes]:
                vehID = veh.split("_")
                carsWaiting[vehID[0]] = traci.vehicle.getAccumulatedWaitingTime(vehID[0])
        
        return carsWaiting

        # RETURNS NUMBER OF CARS WAITING AT AN INTERSECTION
    def carsWaitingCount(self, trafficLight):
        state = self.getState(trafficLight)
        carsWaiting = 0
            # Count all vehicles in the state dictionary
        for lanes in state:
            carsWaiting += len(state[lanes])
        
        return carsWaiting
        
        # RETURNS NORMALIZED THROUGHPUT BY DIVIDING THE THROUGHPUT BY THE TOTAL VEHICLES AT AN INTERSECTION 
    def getThroughputRatio(self, throughput, totalCarsWaiting):
        if totalCarsWaiting == 0:
            return throughput
        else:
            return throughput/totalCarsWaiting
    
        # RETURNS THROUGHPUT OF AN INTERSECTION BASED ON VEHICLES WAITING BEFORE AND AFTER A GIVEN TIME
    def getThroughput(self, trafficLight, carsWaitingBefore, carsWaitingAfter):
        if not carsWaitingBefore:
            return 0
        elif not carsWaitingAfter:
            return len(carsWaitingBefore)
        else:
            carsThrough = { k : carsWaitingBefore[k] for k in set(carsWaitingBefore) - set(carsWaitingAfter) }
            return len(carsThrough)

        # RETURNS THE AGGREGATE WAITING TIME OF CARS THAT HAVE GONE THROUGH THE INTERSECTION AT A GIVEN TIME
    def getThroughputWaitingTime(self, trafficLight, carsWaitingBefore, carsWaitingAfter):
        carsThrough = { k : carsWaitingBefore[k] for k in set(carsWaitingBefore) - set(carsWaitingAfter) }
            
            # Update the relevant individual's aggregate vehicle wait time and return the throughput waiting time
        trafficLight.getAssignedIndividual().updateAggregateVehicleWaitTime(sum(carsThrough.values()))
        return sum(carsThrough.values())        
        
        # RETURNS TOTAL WAIT TIME AT AN INTERSECTION AT A GIVEN TIME
    def getWaitingTime(self, trafficLight):
        waitTime = 0
            # Sum waiting time of each edge controlled by the traffic light
        for edge in trafficLight.getEdges():
            waitTime += traci.edge.getWaitingTime(edge)
        
        return waitTime

        # RETURNS TOTAL WAITING TIME OF A DICTIONARY OF VEHICLES WAITING AT AN INTERSECTION
    def getTotalWaitingTime(self, listOfVehicles):
        return sum(listOfVehicles.values())

        # RETURNS NORMALIZED WAIT TIME REDUCED BY DIVIDING THE WAIT TIMES OF THROUGHPUT VEHICLES BY THE TOTAL WAIT TIME AT AN INTERSECTION 
    def getWaitTimeReducedRatio(self, throughputWaitTime, totalWaitTime):
        if totalWaitTime == 0:
            return 1
        else:
            return throughputWaitTime/totalWaitTime    
    
        # RETURNS RULES THAT ARE APPLICABLE AT A GIVEN TIME AND STATE
    def getValidRules(self, trafficLight, individual):
        validRS = []
        validRSint = []
            
            # Find valid RS rules
        for rule in individual.getRS():
            if self.evaluateRule(trafficLight, rule):
                validRS.append(rule)
            
            # Find valid RSint rules
        for rule in individual.getRSint():
            if self.evaluateCoopRule(trafficLight, rule):
                validRSint.append(rule)

        return (validRS, validRSint)

        # EVALUATE RULE VALIDITY (fEval)
    def evaluateRule(self, trafficLight, rule):
        if rule.getType() == 1:
            return evaluateCoopRule(trafficLight, rule)

            # For each condition, its parameters are acquired and the condition predicate is evaluated
        for cond in rule.getConditions():
            predicateSplit = cond.split("_")
            predicate = predicateSplit[0]

            predCall = getattr(PredicateSet, cond)(self.getPredicateParameters(trafficLight, predicate)) # Construct predicate fuction call
                # Determine validity of predicate
            if predCall == False:
                return False
        
        return True # if all predicates return true, evaluate rule as True

        # EVALUATE RULE VALIDITY (fEval)
    def evaluateCoopRule(self, trafficLight, rule):
        if rule.getType() == 0:
            return self.evaluateRule(trafficLight, rule)
            
        intentions = trafficLight.getCommunicatedIntentions()   

        for x in intentions:
            for i in intentions[x]:
                    # For each condition, its parameters are acquired and the condition predicate is evaluated
                for cond in rule.getConditions():
                    predicateSplit = cond.split("_")
                    predicate = predicateSplit[0]
                    
                    parameters = self.getCoopPredicateParameters(trafficLight, predicate, i)
                    # # print("Parameters are:", parameters)
                    if isinstance(parameters, int) or isinstance(parameters, float) or isinstance(parameters, str):
                        predCall = getattr(CoopPredicateSet, cond)(parameters) # Construct predicate fuction call
                    else:
                        predCall = getattr(CoopPredicateSet, "customPredicate")(parameters[0], parameters[1]) # Construct predicate fuction call for custom predicates (they are of form TLname_action but are handled by the same predicate in CoopPredicateSet)
                        # Determine validity of predicate
                    if predCall == False:
                        return False

        return True # if all predicates return true, evaluate rule as True

        # DETERMINE IF ANY USER DEFINED RULES ARE APPLICABLE
    def applicableUserDefinedRule(self, trafficLight, userDefinedRules):    
            # Evaluate each user define rule
        for rule in userDefinedRules:
                # For each rule, its parameters are acquired and the condition predicate is evaluated
            for cond in rule.getConditions():    
                if "emergencyVehicleApproaching" in cond:
                    continue
                else:
                    parameters = self.getPredicateParameters(trafficLight, cond)
                    predCall = getattr(PredicateSet, cond)(parameters[0], parameters[1], parameters[2]) # Construct predicate fuction call
                    
                    # Determine validity of predicate
                if predCall == True:
                    # # print("User defined rule applicable:", rule.getConditions())
                    return rule
        return False # if no user-defined predicates are applicable, return False

        # APPLIES USER DEFINED ACTIONS
    def applyUserDefinedRuleAction(self, trafficLight, currPhaseName, rule):
            # If max green phase time reached, switch phase to yellow in same direction
        if rule.getConditions()[0] == "maxGreenPhaseTimeReached":
            currPhase = traci.trafficlight.getPhaseName(trafficLight.getName())
            currPhase[5] = "Y"
            traci.trafficlight.setPhase(trafficLight.getName(), currPhase)
            
            # If max yellow phase time reached, switch to next phase in the schedule 
        elif rule.getConditions()[0] == "maxYellowPhaseTimeReached":
            if traci.trafficlight.getPhase(trafficLight.getName()) >= (len(trafficLight.getPhases()) - 2):
                traci.trafficlight.setPhase(trafficLight.getName(), 0)
            else:
                traci.trafficlight.setPhase(trafficLight.getName(), traci.trafficlight.getPhase(trafficLight.getName()) + 1)

        # PROVIDE SIMULATION RELEVANT PARAMETERS
    def getPredicateParameters(self, trafficLight, predicate):
        if predicate == "longestTimeWaitedToProceedStraight":
                # Find max wait time for relevant intersection
            maxWaitTime = 0
            state = self.getState(trafficLight) # Retrieve state of specified intersection 
            for lane in state:
                if lane in trafficLight.getLanes():
                    for veh in state[lane]:
                        if "_S" in veh:
                            vehIDSplit = veh.split("_")
                            vehID  = vehIDSplit[0]
                            if traci.vehicle.getWaitingTime(vehID) > maxWaitTime:
                                maxWaitTime = traci.vehicle.getWaitingTime(vehID)
            return maxWaitTime

        elif predicate == "longestTimeWaitedToTurnLeft":
                # Find max wait time for relevant intersection
            maxWaitTime = 0
            state = self.getState(trafficLight) # Retrieve state of specified intersection 
            for lane in state:
                if lane in trafficLight.getLanes():
                    for veh in state[lane]:
                        if "_L" in veh:
                            vehIDSplit = veh.split("_")
                            vehID  = vehIDSplit[0]
                            if traci.vehicle.getWaitingTime(vehID) > maxWaitTime:
                                maxWaitTime = traci.vehicle.getWaitingTime(vehID)
            return maxWaitTime

        elif predicate == "numCarsWaitingToProceedStraight":
            carsWaiting = 0
            state = self.getState(trafficLight) # Retrieve state of specified intersection 
            for lane in state:
                if lane in trafficLight.getLanes():
                    for veh in state[lane]:
                        if "_S" in veh:
                            vehIDSplit = veh.split("_")
                            vehID  = vehIDSplit[0]
                            if traci.vehicle.getWaitingTime(vehID) > 0:
                                carsWaiting += 1

            return carsWaiting

        elif predicate == "numCarsWaitingToTurnLeft":
            carsWaiting = 0
            state = self.getState(trafficLight) # Retrieve state of specified intersection 
            for lane in state:
                if lane in trafficLight.getLanes():
                    for veh in state[lane]:
                        if "_L" in veh:
                            vehIDSplit = veh.split("_")
                            vehID  = vehIDSplit[0]
                            if traci.vehicle.getWaitingTime(vehID) > 0:
                                carsWaiting += 1

            return carsWaiting
        
        elif predicate == "timeSpentInCurrentPhase":
            return traci.trafficlight.getPhaseDuration(trafficLight.getName())
        
        elif "verticalPhaseIs" in predicate or "horizontalPhaseIs" in predicate or "northSouthPhaseIs" in predicate or "southNorthPhaseIs" in predicate or "eastWestPhaseIs" in predicate or "westEastPhaseIs" in predicate:
            return traci.trafficlight.getPhaseName(trafficLight.getName()).split("_")

        elif "maxGreenPhaseTimeReached" == predicate:
            parameters = []
            parameters.append(traci.trafficlight.getPhaseName(trafficLight.getName()))
            
                # Get phase (G or Y) from phase name
            getPhase = parameters[0].split("_")
            parameters[0] = getPhase[2]
            
            parameters.append(traci.trafficlight.getPhaseDuration(trafficLight.getName()) - (traci.trafficlight.getNextSwitch(trafficLight.getName()) - traci.simulation.getTime()))
            parameters.append(self.maxGreenPhaseTime)

            return parameters
        
        elif "maxYellowPhaseTimeReached" == predicate:
            parameters = []  
            parameters.append(traci.trafficlight.getPhaseName(trafficLight.getName())) # Get traffic light phase name
                
                # Get phase (G or Y) from phase name
            getPhase = parameters[0].split("_")
            parameters[0] = getPhase[2]
            
            parameters.append(traci.trafficlight.getPhaseDuration(trafficLight.getName()) - (traci.trafficlight.getNextSwitch(trafficLight.getName()) - traci.simulation.getTime()))
            parameters.append(self.maxYellowPhaseTime)

            return parameters 
        
        # PROVIDE SIMULATION RELEVANT PARAMETERS
    def getCoopPredicateParameters(self, trafficLight, predicate, intention):        
        if "timeSinceCommunication" == predicate:
            timeSent = intention.getTime()            
            return traci.simulation.getTime() - timeSent
        
        elif "intendedActionIs" == predicate:
            return intention.getAction()
        
        else:       # equivalent to: elif "customPredicate" == predicate:
            # print("The current traffic light is", trafficLight.getName(), "with predicate", predicate, "and an intention from", intention.getTrafficLight(), "to do action", intention.getAction())
            return (str(intention.getTrafficLight().getName()) + "_" + str(intention.getAction()), intention)

        
# main entry point
if __name__ == "__main__":

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "config_file.sumocfg",
                            "--tripinfo-output", "tripinfo.xml"])
    run()

