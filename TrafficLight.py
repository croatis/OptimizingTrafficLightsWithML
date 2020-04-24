import os
import sys
from numpy.random import choice

from Intention import Intention
class TrafficLight:

    global pCoop                    # Probability of choosing rule from RS vs RSint as defined in "Learning cooperative behaviour for the shout-ahead architecture" (2014) 
    global assignedIndividual

    pCoop = 0.5

    def __init__ (self, name, lanes):
        self.name = name
        self.lanes = lanes
        self.edges = []
        self._setEdges(self.lanes)
        self.phases = []
        self.currentRule = None 
        self.carsWaiting = {}
        self.waitTime = 0
        self.doNothingCount = 0
        self.communicationPartners = []
        self.communicatedIntentions = {}
        self.recievedIntentions = {}

        # RETURNS THE TRAFFIC LIGHT'S NAME
    def getName(self):
        return self.name

        # RETURNS THE NUMBER OF LANES CONTROLLED BY THE TRAFFIC LIGHT
    def getLanes(self):
        return self.lanes

        # RETURNS THE NUMBER OF EDGES CONTROLLED BY THE TRAFFIC LIGHT
    def getEdges(self):
        return self.edges
        
        # SETS THE NUMBER OF EDGES CONTROLLED BY THE TRAFFIC LIGHT
    def _setEdges(self, lanes):
        # Determine edges from lanes
        for l in lanes:
            # Isolate edge name from lane name
            edge = l.split("_")
            
            # Ensure edge being added to list isn't a duplicate or retains "LTL" designation
            if edge[1] == "LTL":
                edgeName = edge[0] + "_LTL"
                if edgeName not in self.edges:
                    self.edges.append(edgeName)
            
            elif edge[0] not in self.edges:
                self.edges.append(edge[0])
            
            else:
                print("Edge already exists or is unprocessable:", edge)

       
        # RETURNS THE PHASES AVAILBLE TO THE TRAFFIC LIGHT
    def getPhases(self):
        return self.phases
        
        # SETS THE PHASES AVAILBLE TO THE TRAFFIC LIGHT
    def setPhases(self, phases):
        self.phases = phases

        # SETS THE PHASES AVAILBLE TO THE TRAFFIC LIGHT
    def addPhase(self, phase):
        self.phases.append(phase)
        # print("Adding a new phase to TL. Phases now include:", self.phases)
    
        # RETURN THE CURRENTLY SELECTED RULE
    def getCurrentRule(self):
        return self.currentRule

        # SET THE CURRENTLY SELECTED RULE
    def setCurrentRule(self, rule):
        self.currentRule = rule

        # RETURNS THE AGENT POOL OF THE TRAFFIC LIGHT
    def getAgentPool(self):
        return self.agentPool
        
        # ASSIGNS THE TRAFFIC LIGHT TO AN AGENT POOL
    def assignToAgentPool(self, agentPool):
        self.agentPool = agentPool

        # RETURNS THE RULE SET INDIVIDUAL CURRENTLY BEING USED BY THE TRAFFIC LIGHT FOR A SIM RUN
    def getAssignedIndividual(self):
        return self.assignedIndividual

        # ASSIGNS A RULE SET INDIVIDUAL CURRENTLY BEING USED BY THE TRAFFIC LIGHT FOR A SIM RUN
    def assignIndividual(self):
        self.assignedIndividual = self.agentPool.selectIndividual()
        print("Individual selected is", self.assignedIndividual)
        self.assignedIndividual.selected() # Let Individual know it's been selected

        # RETURNS THE TOTAL NUMBER OF CARS WAITING AT THE TRAFFIC LIGHT'S INTERSECTION
    def getCarsWaiting(self):
        return self.carsWaiting
    
        # SETS THE TOTAL NUMBER OF CARS WAITING AT THE TRAFFIC LIGHT'S INTERSECTION
    def updateCarsWaiting(self, carsWaiting):
        self.carsWaiting = carsWaiting

        # RETURNS THE TOTAL WAIT TIME OF CARS WAITING AT THE TRAFFIC LIGHT'S INTERSECTION
    def getWaitTime(self):
        return self.waitTime
    
        # SETS THE TOTAL WAIT TIME OF CARS WAITING AT THE TRAFFIC LIGHT'S INTERSECTION
    def setWaitTime(self, waitTime):
        self.waitTime = waitTime

        # INCREMENTS THE NUMBER OF TIMES THE TL HAS APPLIED THE Do Nothing ACTION
    def doNothing(self):
        self.doNothingCount += 1
        
        # RETURN THE doNothingCount 
    def getDoNothingCount(self):
        return self.doNothingCount

        # RETURN LIST OF COMMUNICATION PARTNERS
    def getCommunicationPartners(self):
        return self.communicationPartners
        
        # SET LIST OF COMMUNICATION PARTNERS
    def setCommunicationPartners(self, commPartners):
        self.communicationPartners = commPartners

        # SET TL'S NEXT INTENDED ACTION
    def setIntention(self, intention):
        self.communicateIntention(intention)
        self.communicatedIntentions[intention.getTurn()] = intention

        # COMMUNICATE INTENTION TO ALL COMMUNICATION PARTNERS
    def communicateIntention(self, intention):
        for tl in self.communicationPartners:
            tl.recieveIntention(intention)

        # RECIEVE AN INTENTION FROM A COMMUNICATION PARTNER
    def recieveIntention(self, intention):
        if intention.getTurn() not in self.recievedIntentions:
            self.recievedIntentions[intention.getTurn()] = []
        
        self.recievedIntentions[intention.getTurn()].append(intention)
        # print(self.getName(), "recieved an intention from", intention.getTrafficLight().getName(), "\n")

    def getCommunicatedIntentions(self):
        return self.recievedIntentions

        # DECIDE WHICH RULE TO APPLY AT CURRENT ACTION STEP
    def getNextRule(self, validRulesRS, validRulesRSint, time): 
            # First, select a rule from RS and communicate it
        intendedRule = self.getAssignedIndividual().selectRule(validRulesRS)    # Get intended rule to apply

        if intendedRule == -1:
            return -1

        self.setIntention(Intention(self, intendedRule.getAction(), time))
            
            # If intended rule isn't user-defined, select a rule from RSint and then decide between the two
        coopRule = self.getAssignedIndividual().selectCoopRule(validRulesRSint)

            # If no valid rules apply from RSint, return the intented rule from RS
        if coopRule == -1:
            return intendedRule

        if coopRule.getWeight() >= intendedRule.getWeight():
            return coopRule
        else:
            rule = choice([coopRule, intendedRule], 1, p = [pCoop, (1-pCoop)])  # Select one of the two rules based on pCoop value
            return rule[0]                                                      # Choice returns an array, so we take the only element of it

