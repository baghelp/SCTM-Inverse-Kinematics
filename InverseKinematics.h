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

#ifndef HORIZONTAL_SPINE_CONTROLLER_H
#define HORIZONTAL_SPINE_CONTROLLER_H

/**
 * @file HorizontalSpineController.h
 * @brief Contains the definition of class InverseKinematics
 * @author Drew Sabelhaus
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
#include "core/tgTags.h"
#include "core/tgRod.h"
#include "core/tgString.h"
#include "core/tgBox.h"

// The C++ standard library
#include <string>
#include <vector>
#include <map>

//
#include <utility>
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrixX.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgTags.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btQuaternion.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
// #include <ssteam> 
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <string.h>
#include <math.h>
#include "./../../QuadProgpp/src/QuadProg++.hh" 

#include <boost/math/special_functions/round.hpp>


#define GRAVITY 98.1
#define PAYLOAD_MASS 20 // 0.001 kg (to work with YAML density)
// #define PAYLOAD_MASS 10 // kg's (original units)
#define ROD_MASS 6
#define NUM_CABLES 12
#define CROWS 12
#define CCOLS 13
#define MIN_RL 0.5

// printing progress messages
#define VERBOSE false

using namespace std;
using namespace boost::numeric;

// Forward declarations
class TensegrityModel;
class tgBasicActuator;
typedef ublas::matrix<double> myMat;
typedef ublas::vector<double> myVec;

/**
 * A controller to apply the length change in the cables of the 3-bar example
 * model, for the NTRT Introduction Seminar on 2016-09-28 in BEST.
 */
class InverseKinematics : public tgObserver<TensegrityModel>, public tgSubject<InverseKinematics>
{
public:
	
  /**
   * Construct a HorizontalSpineController.
   * @param[in] startTime, a double that determines when the controller
   * begins its motion, how many seconds after the simulation starts.
   * @param[in] minLength, a double that is the percent of the initial length
   * that this controller will reduce down to. E.g., if minLength = 0.25, 
   * controller will act until the rest length of the cables is 25% of initial.
   * @param[in] rate, the rate at which the rest length of the cables will be
   * changed. Expressed in meters/sec.
   * @param[in] tagsToControl, a vector (array) of strings, which is a list of the 
   * tags of all the
   * cables upon which to act. All the cables which have a tag in this list of tags
   * will be acted upon by this controller.
   */
  InverseKinematics(double startTime, double minTension, double rate,
			    double angle, double, double, double);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~InverseKinematics() { 
  }


  /**
   * Apply the controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(TensegrityModel& subject);

  /**
   * Changes the cables' lengths at some specified timestep.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);

  /**
   */
  virtual void moveTheBall(double x, double y, double z );

  /**
   */
  pair<btVector3*, btVector3*> getEnds(typeof(tgBaseRigid)* member);

  /**
   */
  // pair<myMat*, myMat*> QRDecomp( myMat A );

  /**
   */
  myVec proj( myVec, myVec );

  /**
   */
  // template<class T>
  bool InvertMatrix(const myMat* input, myMat* inverse);
// InvertUpperTriangularMatrix

  /**
   */
  myVec findW(myMat V, myMat A_dag, myVec forces);

  /**
   */
  double cost(myMat V, myVec w, myMat A_dag_s, myVec p );


  /**
   */
  void updateNodes();


  /**
   */
  void updateRLengths( btVector3 relativeCubeTarget );

protected:

  /**
   * A helper function to find the structural elements of the given tensegrity
   * object
   * @param[in] subject, the tensegrity object to be initialized
   */
  void initializeStructure( TensegrityModel& subject );
    
  /**
   * A helper function to setup Conectivity matrix
   */
  void initializeController( );

  /**
   * Return a matrix with the passed in vector as its diagonal elements, and
   * zeros in the off diagonals
   */
  myMat diag( myVec inVec );

  /**
   * get the center of the current superball
   */
  btVector3 getSuperballCenter();

  /**
   * use inverse kinematics to find the necessary adjustment to the cables
   * lengths
   */
  myVec inverseKin( myVec targetActualLengths, int C[12][13], int Crows, int Ccols, double springConstant, vector<btVector3*> nodes, btVector3 target );

private:
	
  /**
   * The private variables for each of the values passed in to the constructor.
   */
  double startTime;
  double minTension;
  double rate;
  double angleOfTravel;
  double cubeXOffset;
  double cubeYOffset;
  double cubeZOffset;
  std::vector<tgBasicActuator*> cables;
  std::vector<btVector3> desiredCableDirections = std::vector<btVector3>();
  std::vector<tgRod*> rods;
  std::vector<tgBox*> cube;
  std::vector< btVector3* > nodes;
  std::vector< btVector3 > initDisplacementVectors = std::vector<btVector3>();
  // setting up vectors for rope offsets (from cubeCenter to attachment points)
  // under various cube orientations
  btVector3 cubeCenter = btVector3( 0, 8.42, 0);
  // // tfm
  // initDisplacementVectors.push_back( btVector3(-1.27,   9.69, 0) - cubeCenter);
  // // bfm
  // initDisplacementVectors.push_back( btVector3(-1.27,   7.15, 0) - cubeCenter);
  // // tbm
  // initDisplacementVectors.push_back( btVector3(1.27,   9.69, 0) - cubeCenter);
  // // bbm
  // initDisplacementVectors.push_back( btVector3(1.27,   7.15, 0) - cubeCenter);
  // // bmr
  // initDisplacementVectors.push_back( btVector3(0,   7.15, 1.27) - cubeCenter);
  // // bml
  // initDisplacementVectors.push_back( btVector3(0,   7.15, -1.27) - cubeCenter);
  // // tmr
  // initDisplacementVectors.push_back( btVector3(0,   9.69, 1.27) - cubeCenter); 
  // // tml
  // initDisplacementVectors.push_back( btVector3(0,   9.69, -1.27) - cubeCenter);
  // // mbl
  // initDisplacementVectors.push_back( btVector3(1.27, 8.42, -1.27) - cubeCenter);
  // // mfl
  // initDisplacementVectors.push_back( btVector3(-1.27,8.42, -1.27) - cubeCenter);
  // // mbr
  // initDisplacementVectors.push_back( btVector3(1.27,  8.42, 1.27) - cubeCenter);
  // // mfr
  // initDisplacementVectors.push_back( btVector3(-1.27, 8.42, 1.27) - cubeCenter);

  std::vector< btVector3 > currDisplacementVectors = vector<btVector3>();

  std::vector< btVector3* > initialNodes = vector<btVector3*>();
  std::string const CABLES[NUM_CABLES] = {"cube_string_1", "cube_string_2",
    "cube_string_3", "cube_string_4", "cube_string_5", "cube_string_6", 
    "cube_string_7", "cube_string_8", "cube_string_9", "cube_string_10", 
    "cube_string_11", "cube_string_12"};
  std::string const RODS[6] = {"rod_1", "rod_2", "rod_3", "rod_4", "rod_5",
    "rod_6"};
  std::string const CUBE = "super_cube";
  double springConstant = 613.0; // TODO: assuming K = stiffness
  //1.0/613.0; // TODO: assuming K = 1/stiffness. check this.
  myVec targetLengths = myVec(NUM_CABLES);
  myVec targetRLengths = myVec(targetLengths.size());
  btVector3 relativeCubeTarget;
  btVector3 ballCenter;
  btVector3* middle;
  //still unsure of units and unsure if ntrt even has the correct cable
  //stiffness for our cables
  //
  // rows = cables, colums = nodes. connections are 1 or -1
  // (-1 is the cube connection, 1 is the rod connection)
  int C[CROWS][CCOLS] = {
  // 1   2   3   4   5   6   7   8   9   0   1   2   1  
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1 }, 
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1 },
    {0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1 },
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0, -1 },
    {0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0, -1 },
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0, -1 },
    {0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0, -1 },
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0, -1 },
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0, -1 },
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0, -1 },
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0, -1 },
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1, -1 }
  };
  
  ublas::matrix<double> Connectivity;// = myMat(12, 13);
  ublas::vector<double> A;
  // btMatrixX<double> Connectivity;// = myMat(12, 13);
  // btMatrixX<double> A;// = myMat(12, 13);

  int extForces[3][13] = {
  //                                   |
  //1  2  3  4  5  6  7  8  9  0  1  2  1
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  -GRAVITY*PAYLOAD_MASS}
  };

  //std::int Connectivity[13][24] = {
  // //                                   |
  // //1  2  3  4  5  6  7  8  9  0  1  2  1  2  3  4  5  6  7  8  9  0  1  2
  //   1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0  0  0  0  0  0  0 // 1 -- rope
  //   0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0  0  0  0  0  0 // 2 -- rope
  //   0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0  0  0  0  0 // 3 -- ...
  //   0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0  0  0  0 // 4
  //   0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0  0  0 // 5
  //   0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0  0 // 6
  //   0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0  0 // 7
  //   0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0  0 // 8
  //   0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0  0 // 9
  //   0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0  0 // 10
  //   0  0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1  0 // 11
  //   0  0  0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0 -1 // 12 -- rope
  //   0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0 // 13 -- cube

    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 2
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 3
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 4
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 5
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 6
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 7
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 8
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 9
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 10 
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 11
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 12
    // 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 // 13

  /**
   * Need an accumulator variable to determine when to start the controller.
   */
  double timePassed;

  /**
   * The start length of each of the cables must be recorded.
   * This map takes a string (the space-separated list of all the tags for
   * an individual cable) and outputs a double (the rest length at time t=0.)
   */
  typedef std::map<tgTags, double> InitialRestLengths;
  InitialRestLengths initialRL;

  /**
   * A list of all the actuators to control. This is populated in onSetup
   * by using tagsToControl.
   */
  /*
  std::vector<tgBasicActuator*> cablesWithTags;
  std::vector<tgBasicActuator*> lengthen_vector;
  std::vector<tgBasicActuator*> shorten_vector;
  */

};

#endif // HORIZONTAL_SPINE_CONTROLLER_H
