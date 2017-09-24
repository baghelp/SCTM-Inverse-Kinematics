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
 * @file Cubetroller.cpp -- McRandy playing with machine learning
 * @brief Implementation of Cubetroller.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "Cubetroller.h"
// // This application
// #include "yamlbuilder/TensegrityModel.h"
// // This library
// #include "core/tgBasicActuator.h"
// #include "core/tgSpringCableActuator.h"
// #include "core/tgTags.h"
// #include "LinearMath/btVector3.h"
// #include "LinearMath/btScalar.h"
// #include "LinearMath/btMatrix3x3.h"
// #include "LinearMath/btQuaternion.h"
//
// //#include "sensors/tgDataObserver.h"
// // The C++ Standard Library
// #include <cassert>
// #include <stdexcept>
// #include <vector>
// #include <iostream>
// #include <ssteam>
// #include "helpers/FileHelpers.h"
// #include <stdexcept>
// #include <string.h>
// #include <math.h>




// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
Cubetroller::Cubetroller(double minTension,
    double rate,
    double angleOfTravel) {
  //vector<string> rods,
  //vector<string> strings) :
  // START_TIME(START_TIME),
  // minTension(minTension),
  // rate(rate),
  // angleOfTravel(angleOfTravel),
  // timePassed(0.0)
  //{
  this->minTension = minTension;
  this->rate = rate;
  this->angleOfTravel = angleOfTravel;
  this->timePassed =  0.0;
  // minTension must be greater than some lower bound
  if( minTension <= 0 ) {
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
void Cubetroller::initializeStructure( TensegrityModel& subject ) {
  cout<< "initializing stucture... ";
  // Pick out the actuators and rods by tag
  cube.erase(cube.begin(), cube.end());
  rods.erase(rods.begin(), rods.end());
  cables.erase(cables.begin(), cables.end());

  // for each name in CABLES, add the corresponding cable to the cables array
  for( int i = 0; i < sizeof(CABLES)/sizeof(CABLES[0]); i++ ) {
    string tag = CABLES[i];
    tgBasicActuator* grabbedCable = subject.find<tgBasicActuator>(tag).front();
    cables.push_back(grabbedCable);
  }
  // for each name in RODS, add the corresponding rod to the rods array
  for( int i = 0; i < sizeof(RODS)/sizeof(RODS[0]); i++ ) {
    string tag = RODS[i];
    tgRod* grabbedRod = subject.find<tgRod>(tag).front();
    rods.push_back(grabbedRod);
  }
  // grab the box, add it to the cube array
  string tag = CUBE;
  tgBox* box = subject.find<tgBox>(tag).front();
  cube.push_back( box );


  // fill the nodes vector with vectors representing the rod end points.
  updateNodes( );
  cout<< " Done. " <<endl;
}


void Cubetroller::updateNodes() {
  nodes.erase(nodes.begin(), nodes.end() );
  //for each rod in the RODS, add its end points to the nodes vector
  for( size_t i = 0; i < rods.size(); i++ ) {
    pair< btVector3*, btVector3* > endPoints = rods[i]->ends();
    nodes.insert( nodes.begin(), endPoints.first );
    nodes.insert( nodes.begin(), endPoints.second );
  }

  //add the middle of the cube to the nodes vector
  btVector3 top = *(cube[0]->ends().first);
  btVector3 bottom = *(cube[0]->ends().second);
  btVector3* middle = new btVector3();
  *middle = (top + bottom)/2;
  nodes.insert( nodes.begin(), middle );
  // for( int i = 0; i < nodes.size(); i++ ) {
  //   // cout<< *nodes[i] << ", ";
  // }
  // cout<<endl;
  return;
}


/**
 * For this controller, the onSetup method initializes the structure (finds the
 * rods and cables), and initializes the controller (reads theta values from
 * file, and sets a target direction of travel
 */
void Cubetroller::onSetup( TensegrityModel& subject )
{
  cout << "Setting up the Cubetroller controller." << endl;
  //	    << "Finding cables with tags: " << lengthen
  //	    << endl;
  //lengthen_vector = {};
  //shorten_vector = {};
  initializeStructure( subject );
  initializeController();
  // For all the strings in the list, call initializeActuators.
  cout << "Finished setting up the controller." << endl;    
}


void Cubetroller::initializeController() {
  cout<< "initializing controller " <<endl;
  // thetarray[0][0] = 0;
  // for( int row = 0; row < 4; row++ ) {
  //   for( int col = 0; col < 12; col++ ) {
  //     theta(row, col) = thetarray[row][col];
  //   }
  // }
  // for( int row = 0; row < 26; row++ ) {
  //   for( int col = 0; col < 4; col++ ) {
  //     Theta1(row, col) = Theta1array[row][col];
  //   }
  // }
  // for( int row = 0; row < 12; row++ ) {
  //   for( int col = 0; col < 26; col++ ) {
  //     Theta2(row, col) = Theta2array[row][col];
  //   }
  // }

  // Move Superball!
  // read theta from file
  readMatFiles();
  // set a target direction of travel
  targetDeltaCOM(0) = 1;
  targetDeltaCOM(1) = 5;//7; // horizontal
  targetDeltaCOM(2) = 2; // vertical
  targetDeltaCOM(3) = -5; // horizontal
  cout<< "finished initializing controller" <<endl;
  return;
}


/* Read thetas from files, so that you don't have to put tons of constants in
 * the h file
 */
void Cubetroller::readMatFiles() {
  cout<< "reading in theta and intercept files"<<endl;
  readTheta( Theta1 , THETA0_FILE);
  readTheta( Theta2 , THETA1_FILE);
  readTheta( Theta3 , THETA2_FILE);
  readTheta( Theta4 , THETA3_FILE);
  readTheta( Theta5 , THETA4_FILE);
  readTheta( Theta6 , THETA5_FILE);
  readTheta( Theta7 , THETA6_FILE);


  readIntercept( Intercept1 , INTERCEPT0_FILE);
  readIntercept( Intercept2 , INTERCEPT1_FILE);
  readIntercept( Intercept3 , INTERCEPT2_FILE);
  readIntercept( Intercept4 , INTERCEPT3_FILE);
  readIntercept( Intercept5 , INTERCEPT4_FILE);
  readIntercept( Intercept6 , INTERCEPT5_FILE);
  readIntercept( Intercept7 , INTERCEPT6_FILE);

  // cout<< "Theta1"<<endl ;
  // for( int row = 0; row < Theta1.size1(); row++ ) {
  //   for( int col = 0; col < Theta1.size2(); col++ ) {
  //     cout<< Theta1(row, col) << ", ";
  //   }
  //   cout <<endl;
  // }
  // for( int row = 0; row < 2; row++ ) {
  //   for( int col = 0; col < 3; col++ ) {
  //     cout<< array[row][col];
  //   }
  // }

  // inFile.open(THETA2_FILE);
  // if( !inFile) {
  //   cout << "Unable to open file: theta2"<<endl;
  // }
  //
  // i = 0;
  // matCols = Theta2.size2();
  // while (inFile >> num) {
  //   Theta2((int)i/matCols, (int)i%matCols) = num;
  //   i++;
  // }
  //
  // inFile.close();
  // // for( int row = 0; row < 2; row++ ) {
  // //   for( int col = 0; col < 3; col++ ) {
  // //     cout<< array[row][col];
  // //   }
  // // }
  //
  //
  // inFile.open(INTERCEPT1_FILE);
  // if( !inFile) {
  //   cout << "Unable to open file: intercept1"<<endl;
  // }
  //
  // i = 0;
  // while (inFile >> num) {
  //   Intercept1(i) = num;
  //   i++;
  // }
  //
  // inFile.close();
  // // for( int row = 0; row < 2; row++ ) {
  // //   for( int col = 0; col < 3; col++ ) {
  // //     cout<< array[row][col];
  // //   }
  // // }
  //
  //
  // inFile.open(INTERCEPT2_FILE);
  // if( !inFile) {
  //   cout << "Unable to open file: intercept2"<<endl;
  // }
  //
  // i = 0;
  // while (inFile >> num) {
  //   Intercept2(i) = num;
  //   i++;
  // }
  //
  // inFile.close();
  // // for( int row = 0; row < 2; row++ ) {
  // //   for( int col = 0; col < 3; col++ ) {
  // //     cout<< array[row][col];
  // //   }
  // // }
  //
  //
  //
  cout<< "finished reading files"<<endl;

}

void Cubetroller::readTheta( mat& theta, string thetaFile ) {
  double num;
  ifstream inFile;
  inFile.open( thetaFile.c_str() );
  if( !inFile) {
    cout << "Unable to open file: "<<thetaFile<<endl;
  }

  int i = 0;
  int matCols = theta.size2();
  while (inFile >> num) {
    theta((int)i/matCols, (int)i%matCols) = num;
    i++;
  }

  inFile.close();
  return;
}


void Cubetroller::readIntercept( vec& intercept, string interceptFile ) {
  double num;
  ifstream inFile;
  inFile.open( interceptFile.c_str() );
  if( !inFile) {
    cout << "Unable to open file: "<<interceptFile<<endl;
  }

  int i = 0;
  while (inFile >> num) {
    intercept(i) = num;
    i++;
  }

  inFile.close();
  return;
}


/* Calculate RLengths based on desired movement of COM
*/
void Cubetroller::updateRLengths() {
  // linear regression
  // targetRLengths = prod(targetDeltaCOM, theta);
  updateNodes();

  // neural net
  vec inputVec = vec(7);
  int index = 0;
  inputVec(index++) = 1;
  for( int i = 1; i < targetDeltaCOM.size(); i++ ) {
    inputVec(index++) = targetDeltaCOM(i);
  }
  // the vector between the given rodend and the current COM
  btVector3 diff;
  // for( int i = 0; i < NUM_CABLES; i++ ) {
  //   diff = *(nodes[i])- getModelCOM(); // TODO: why is the first node's X coordinate 45 off of the COM?
  //   inputVec(index++) = diff.getX();
  //   inputVec(index++) = diff.getY();
  //   inputVec(index++) = diff.getZ();
  // }
  inputVec(index++) = cube[0]->orientation().getX();
  inputVec(index++) = cube[0]->orientation().getY();
  inputVec(index++) = cube[0]->orientation().getZ();
  cout<< "feature vector" << inputVec <<endl;
  //TODO: check to make sure this output is the right size and is correct
  vec a1 = ReLU(prod(inputVec, Theta1 ) + Intercept1);
  // cout<< "a1: "<<a1<<endl;
  // cout<<endl<<endl<< "intercept1: "<<Intercept1<<endl<<endl;
  // cout<< "Theta1: "<<Theta1<<endl<<endl;
  vec a2 = ReLU( prod(a1, Theta2 ) + Intercept2);
  vec a3 = ReLU( prod(a2, Theta3 ) + Intercept3);
  vec a4 = ReLU( prod(a3, Theta4 ) + Intercept4);
  vec a5 = ReLU( prod(a4, Theta5 ) + Intercept5);
  vec a6 = ReLU( prod(a5, Theta6 ) + Intercept6);
  targetRLengths  = ReLU( prod(a6, Theta7 ) + Intercept7);
  cout<<"here"<<endl;


  // get rid of negative rest lengths
  // for( int i = 0; i < targetRLengths.size(); i++ ) {
  //   if( targetRLengths(i) < 0 ) {
  //     targetRLengths(i) = 0;
  //   }
  // }
  cout<< "Calculated RLengths: "<<targetRLengths<<endl;
  return;
}

vec Cubetroller::ReLU( vec ai ) {
  vec reluOutput( ai.size() );
  for( int i = 0; i < ai.size(); i++ ) {
    if( ai(i) > 0 ) {
      reluOutput(i) = ai(i);
    }
    else {
      reluOutput(i) = 0;
    }
  }

  return reluOutput;
}



/* old playing around-thing, tightens some cables and lengthens others.
 * maintains a min tension as well
 */
void Cubetroller::onStep(TensegrityModel& subject, double dt)
{
  double nextRestLength;
  double currRestLength;
  double minRestLength;
  timePassed += dt;
  step++;

  if( timePassed < START_TIME ) {
    // setup superball on triangular face
    for (size_t i = 0; i < cables.size(); i ++) {
      cables[i]->setControlInput(triangleStart[i], dt);
    }
  }

  else{
    // updateRLengths();
    if( (timePassed - START_TIME) <dt) {
      // find the desired rLengths
      // updateNodes();
      updateRLengths();
      initialCOM = getModelCOM();
      cout<< "sending commands" << endl;
    }

    // send first commands
    // if( timePassed < START_TIME*3 ) {
      if( (int) step % 5000 == 0 ) {
        // updateNodes();
        // updateRLengths();
      }
      for( int i = 0; i < cables.size(); i++ ) {
        // if( cables[i]->getTension() < MIN_TENSION ) {
        //   targetRLengths[i] -= RL_STEP_SIZE;
        // }
        cables[i]->setControlInput(targetRLengths(i), dt);
      }
      if( step %70000 ==0 ) {
        for( int i = 0; i < NUM_CABLES; i++ ) {
          cout<< "cable "<<i<<": "<<cables[i]->getRestLength()<<endl;
        }
        cout<< "deltaCOM: " << getModelCOM() - initialCOM<<endl;
      }

    // }

    // // recenter cube
    // else if( timePassed < START_TIME*5 ) {
    //   if( (int) step % 5000 == 0 ) {
    //     // updateNodes();
    //     // updateRLengths();
    //     for( int i = 0; i < NUM_CABLES; i++ ) {
    //       targetRLengths(i) = CENTR_RL;
    //     }
    //   }
    //   for( int i = 0; i < cables.size(); i++ ) {
    //     if( cables[i]->getTension() < MIN_TENSION ) {
    //       targetRLengths[i] -= RL_STEP_SIZE;
    //     }
    //     cables[i]->setControlInput(targetRLengths(i), dt);
    //   }
    // }
    //
    // // send second commands
    // else if( timePassed < START_TIME*7 ) {
    //   if( (int) step % 5000 == 0 ) {
    //     // updateNodes();
    //     updateRLengths();
    //   }
    //   for( int i = 0; i < cables.size(); i++ ) {
    //     if( cables[i]->getTension() < MIN_TENSION ) {
    //       targetRLengths[i] -= RL_STEP_SIZE;
    //     }
    //     cables[i]->setControlInput(targetRLengths(i), dt);
    //   }
    // }
  }
}


/* Get COM of model
*/
btVector3 Cubetroller::getModelCOM() {
  btVector3 COM_new = btVector3(0, 0, 0);
  double totalMass = 0.0;
  for( int i = 0; i < sizeof(RODS)/sizeof(RODS[0]); i++ ) {
    COM_new += rods[i]->centerOfMass()*ROD_MASS;
    totalMass += ROD_MASS;
  }
  COM_new += cube[0]->centerOfMass()*PAYLOAD_MASS;
  totalMass += PAYLOAD_MASS;
  COM_new /= totalMass;
  return COM_new;
}


/* make a diagonal matrix out of a vector
*/
mat Cubetroller::diag( vec inVec ) {

  mat diagonal = mat( inVec.size(), inVec.size() );

  for( int row = 0; row < inVec.size(); row++ ) {
    for( int col = 0; col < inVec.size(); col++ ) {
      diagonal( row, col ) = 0;
    }
  }
  for( int index = 0; index < inVec.size(); index++ ) {
    diagonal( index, index ) = inVec(index);
  }

  return diagonal;
}

/* Find the projection of vector a onto vetor e
*/
vec Cubetroller::proj(vec a, vec e) { // projection of vector A onto e
  vec result = e*( inner_prod(e, a) / (inner_prod(e, e) ) );
  return result;
}


pair<mat, mat> Cubetroller::grammyDecomp( mat A ) {
  // cout<< "intput to decomp: " << A <<endl;
  mat U = mat(A.size1(), A.size2() );
  mat E = mat(A.size1(), A.size2() );

  // cout<< "loops ends when col = " << A.size1()
  for( int col = 0; col < A.size2(); col++ ) {
    // cout<< "col is "<<col<<endl;
    ublas::column(U, col) = ublas::column(A, col);
    for( int j = 0; j < col; j++ ) {
      // cout<< "j is "<<j<<endl;
      ublas::column(U, col) -= proj( ublas::column(A, col), ublas::column(U, j));
      // cout<< "proj works"<<endl;
    }

    ublas::column(E, col) = ublas::column(U, col) / norm_2( ublas::column(U, col) );
  }
  mat Q = E;
  mat R = prod(ublas::trans(Q),A);
  // cout<< "U: " <<U << endl;
  // cout<< "Q: " <<Q << endl;
  // cout<< "R: " <<R << endl;
  return pair<mat, mat>(Q, R);
}


bool Cubetroller::InvertMatrix(const mat* input, mat* inverse)
{
  typedef ublas::permutation_matrix<std::size_t> pmatrix;

  // create a working copy of the input
  mat A(*input);

  // create a permutation matrix for the LU-factorization
  pmatrix pm(A.size1());

  // perform LU-factorization
  int res = lu_factorize(A, pm);
  if (res != 0) {
    return false;
  }
  inverse->assign(ublas::identity_matrix<double>(A.size1(), A.size2()));

  // backsubstitute to get the inverse
  lu_substitute(A, pm, *inverse);

  return true;
}


