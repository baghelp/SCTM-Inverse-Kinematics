/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */

/**
 * @file LengthControllerYAML.cpp
 * @brief Implementation of LengthControllerYAML.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "LengthControllerYAML.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgTags.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <string.h>
#include <math.h>


using namespace std;



// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
LengthControllerYAML::LengthControllerYAML(double startTime,
					   double minTension,
					   double rate,
             double angleOfTravel): 
					   //vector<string> rods,
					   //vector<string> strings) :
  startTime(startTime),
  minTension(minTension),
  rate(rate),
  angleOfTravel(angleOfTravel),
  timePassed(0.0)
{
  // start time must be greater than or equal to zero
  if( startTime < 0.0 ) {
    throw invalid_argument("Start time must be greater than or equal to zero.");
  }
  // minTension must be greater than some lower bound
  else if( minTension <= 0 ) {
    throw invalid_argument(
      "Tension is not high enough. min is currently set to 1." );
  }
  // rate must be greater than zero
  else if( rate < 0.0 ) {
    throw invalid_argument("Rate cannot be negative.");
  }
  // @TODO: what checks to make on tags?
}

/**
 * The initializeActuators method is called in onSetup to fill the cables and
 * rods arrays, as well as store the initial rest lengths in the initialRL map.
 */
void LengthControllerYAML::initializeActuators( TensegrityModel& subject ) {
  //DEBUGGING
  // Pick out the actuators and rods by tag
  cout << "Finding " <<sizeof(CABLES)/sizeof(CABLES[0])<< " cables..." << endl;

  for( int i = 0; i < sizeof(CABLES)/sizeof(CABLES[0]); i++ ) {
    string tag = CABLES[i];
    cout<< "finding " << tag <<endl;
    vector<tgBasicActuator*> grabbedCables = subject.find<tgBasicActuator>(tag);
    cables.insert( cables.end(), grabbedCables.begin(), grabbedCables.end() );
    cout<< grabbedCables.size()<<endl;
    //cables.push_back( subject.find<tgBasicActuator>(tag) );
  }
  cout<< "# cables found: " << cables.size()<<endl;
  cout << "Finding " <<sizeof(RODS)/sizeof(RODS[0])<< " rods..." << endl;
  for( int i = 0; i < sizeof(RODS)/sizeof(RODS[0]); i++ ) {
    string tag = RODS[i];
    cout<< "finding " << tag <<endl;
    vector<tgRod*> grabbedRods = subject.find<tgRod>(tag);
    rods.insert( rods.end(), grabbedRods.begin(), grabbedRods.end() );
    cout<< grabbedRods.size()<<endl;
  }

  // for debugging -- print out size of cable array, and each element's toString
  cout<< "# cables: " <<cables.size()<< endl;
  for( size_t i = 0; i < cables.size(); i++ ) {
  //for( tgBasicActuator* cable : cables ) {
    cout<<"test"<<endl;
    cout<< cables[i]->toString() << endl;
    //cout<< cable->toString() << endl;
    
  }
  cout<< "end of cables"<<endl;

  // for debugging -- print out size of rods array, and each element's toString
  cout<< "# rods: " <<rods.size()<< endl;
  for( size_t i = 0; i < rods.size(); i++ ) {
    cout<< rods[i]->toString() << endl;
  }
  cout<< "end of rods"<<endl;
  //cout << "The following cables were found and will be controlled: "
	    //<< endl;
  //Iterate through array and output strings to command line
  //
  /*
  for (size_t i = 0; i < foundActuators.size(); i ++) {	
    cout << foundActuators[i]->getTags() << endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    cout << "Cable rest length at t=0 is "
	      << initialRL[foundActuators[i]->getTags()] << endl;
  }
  */
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  //if(foundActuators.begin()./foundAc
  
  //storage_vector.insert( storage_vector.end(), foundActuators.begin(),
			 //foundActuators.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void LengthControllerYAML::onSetup(TensegrityModel& subject)
{
  cout << "Setting up the LengthControllerYAML controller." << endl;
  //	    << "Finding cables with tags: " << lengthen
  //	    << endl;
  //lengthen_vector = {};
  //shorten_vector = {};
  initializeActuators(subject);
  // For all the strings in the list, call initializeActuators.
  cout << "Finished setting up the controller." << endl;    
}

/* old playing around-thing, tightens some cables and lengthens others.
 * maintains a min tension as well
 */
void LengthControllerYAML::onStep(TensegrityModel& subject, double dt)
{
  /*
  double nextRestLength;
  double currRestLength;
  double minRestLength;
  timePassed += dt;
  if( timePassed > startTime ) {
    // if enough time has passed, actuate cables
    for (size_t i = 0; i < shorten_vector.size(); i ++) {	
      // shorten the cables in shorten_vector
      currRestLength = shorten_vector[i]->getRestLength();
      // Calculate the minimum rest length for this cable.
      // Remember that minLength is a percent.
      minRestLength = initialRL[shorten_vector[i]->getTags()] * minLength;
      // If the current rest length is not too small
      if( currRestLength > minRestLength ) {
        // output a progress bar for the controller, to track when control occurs.
        cout << ".";
        nextRestLength = currRestLength - rate * dt;
        //DEBUGGING
        //cout << "Next Rest Length: " << nextRestLength << endl;
        shorten_vector[i]->setControlInput(nextRestLength,dt);
      }
    }   
    for (size_t i = 0; i < lengthen_vector.size(); i ++) {	
      // lengthen the cables in lengthen_vector
      currRestLength = lengthen_vector[i]->getRestLength();
      // Calculate the minimum rest length for this cable.
      // Remember that minLength is a percent.
      minRestLength = initialRL[lengthen_vector[i]->getTags()] * minLength;
      // If the current rest length is not too small
      if( lengthen_vector[i]->getTension() > 1 ){
        // output a progress bar for the controller, to track when control occurs.
        cout << "lengthen";
        nextRestLength = currRestLength + rate * dt;
        //DEBUGGING
        //cout << "Next Rest Length: " << nextRestLength << endl;
        lengthen_vector[i]->setControlInput(nextRestLength,dt);
      }
      else if( currRestLength > minRestLength) {
        cout << "shorten";
        nextRestLength = currRestLength - rate * dt;
        //DEBUGGING
        //cout << "Next Rest Length: " << nextRestLength << endl;
        lengthen_vector[i]->setControlInput(nextRestLength,dt);
      }

    }   
  }
  */
}
// */
	
 
