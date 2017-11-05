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
 * @file BallDropperYAML.cpp
 * @brief Implementation of BallDropperYAML.
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "BallDropperYAML.h"
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


#include "armadillo"

using namespace arma;

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.
BallDropperYAML::BallDropperYAML(double startTime,
		double minTension,
		double rate,
		double angleOfTravel) {
	//vector<string> rods,
	//vector<string> strings) :
	// startTime(startTime),
	// minTension(minTension),
	// rate(rate),
	// angleOfTravel(angleOfTravel),
	// timePassed(0.0)
	//{
	this->startTime = startTime;
	this->minTension = minTension;
	this->rate = rate;
	this->angleOfTravel = angleOfTravel;
	this->timePassed =  0.0;
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
void BallDropperYAML::initializeStructure( TensegrityModel& subject ) {
	//DEBUGGING
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
	// grab the box
	string tag = CUBE;
	tgBox* box = subject.find<tgBox>(tag).front();
  cube.push_back( box );
  middle = new btVector3();

  updateNodes();
  for( int i = 0; i < nodes.size(); i++ ) {
    initialNodes.push_back( new btVector3( *nodes[i]) );
    // cout<< "node " <<i <<": " <<*(initialNodes[i])<<endl;
  }
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void BallDropperYAML::onSetup( TensegrityModel& subject )
{
	cout << "Setting up the BallDropperYAML controller." << endl;
	//	    << "Finding cables with tags: " << lengthen
	//	    << endl;
	//lengthen_vector = {};
	//shorten_vector = {};
	initializeStructure( subject );
	initializeController();
	// For all the strings in the list, call initializeActuators.
	cout << "Finished setting up the controller." << endl;    

}


void BallDropperYAML::moveTheBall( double x, double y, double z ) {
	// 1.create connectivity matrix, calling from and to to get points and stuff
	int rodsAndCables = 12 + 6;
	int nodes = 12 + 12;
	double Connectivity[nodes][rodsAndCables];
}

// #<{(|*
//  * Takes in a pointer to a rod object, then accesses the center of mass, the
//  * orientation, and the length, and then calculates and returns the current
//  * rod end points
//  |)}>#
// pair< btVector3*, btVector3* > BallDropperYAML::getEnds( tgBaseRigid* member ) {
//   //still borken
//   // if( typeof(member) == tgRod* ) {
//   //   member
//   double memberLength = ((tgRod*) member)->length();
//   btVector3 center = member->centerOfMass();
//   btVector3 orientation = member->orientation();
//   btVector3 memberAxis = btVector3( btScalar(0), btScalar(1), btScalar(0) );
//
//   // use btMatrix to avoid the implementation of euler angles, because they are
//   // undocumented and possibly wrong (?)
//   btMatrix3x3 rotMatrix = btMatrix3x3();
//   btQuaternion quatOrientation;
//   rotMatrix.setEulerYPR(orientation.getX(), orientation.getY(),
//       orientation.getZ() ); // fill rotMatrix
//   // set quatOrientation as the quaternion representing the body's orientation
//   rotMatrix.getRotation( quatOrientation ); 
//
//   // Rotate the axis from its initialization to its current orientation
//   btVector3 axisOfRot = quatOrientation.getAxis();
//   btScalar angleOfRot = quatOrientation.getAngle();
//   memberAxis = memberAxis.rotate( axisOfRot, angleOfRot );
//
//   // find endpoints of member
//   btVector3* pt1 = new btVector3();
//   btVector3* pt2 = new btVector3();
//   *pt1 = memberAxis*(memberLength/2) + center;
//   *pt2 = memberAxis*(-memberLength/2) + center;
//
//   //debugging
//   cout << "Center: " << center  << endl;
//   cout << "Rotated pt1: " << *pt1  << endl;
//   cout << "Rotated pt2: " << *pt2  << endl;
//   cout<<endl;
//
//
//   return pair< btVector3*, btVector3* >(pt1, pt2);
// }


/* initialize displacement vectors, and set the value of the global
 * "relativeCubeTarget" (which is taken relative to the center of the superball
 */
void BallDropperYAML::initializeController() {
  cout<< "initializing Controller"<<endl;

  
  // These are really ugly, hardcoded like this, but we don't know how to get
  // the locations of nodes in the yaml file into the cpp program :(
  // mfr
  initDisplacementVectors.push_back( btVector3(-1.27, 8.42, 1.27) - cubeCenter);
  // mbr
  initDisplacementVectors.push_back( btVector3(1.27,  8.42, 1.27) - cubeCenter);
  // mfl
  initDisplacementVectors.push_back( btVector3(-1.27,8.42, -1.27) - cubeCenter);
  // mbl
  initDisplacementVectors.push_back( btVector3(1.27, 8.42, -1.27) - cubeCenter);
  // tml
  initDisplacementVectors.push_back( btVector3(0,   9.69, -1.27) - cubeCenter);
  // tmr
  initDisplacementVectors.push_back( btVector3(0,   9.69, 1.27) - cubeCenter);
  // bml
  initDisplacementVectors.push_back( btVector3(0,   7.15, -1.27) - cubeCenter);
  // bmr
  initDisplacementVectors.push_back( btVector3(0,   7.15, 1.27) - cubeCenter);
  // bbm
  initDisplacementVectors.push_back( btVector3(1.27,   7.15, 0) - cubeCenter);
  // tbm
  initDisplacementVectors.push_back( btVector3(1.27,   9.69, 0) - cubeCenter);
  // bfm
  initDisplacementVectors.push_back( btVector3(-1.27,   7.15, 0) - cubeCenter);
  // tfm
  initDisplacementVectors.push_back( btVector3(-1.27,   9.69, 0) - cubeCenter);


	// btVector3 target = btVector3( btScalar(4), btScalar(8.42), btScalar(0) );
  //     move       alllll      Assign location here
	//          (camera right) (away from ground) (away from camera)       
  // middle = (0. 8.42, 0);
	relativeCubeTarget = btVector3( btScalar(0), btScalar(0), btScalar(0) );

  cout<< "finished initializing Controller"<<endl;
}


void BallDropperYAML::inverseKin( myVec targetActualLengths, int C[12][13], int Crows, int Ccols, double springConstant, vector<btVector3*> nodes, btVector3 target){

	Connectivity = ublas::matrix<double>(Crows, Ccols);
	for( int row = 0; row < Crows; row++ ) {
		for( int col = 0; col < Ccols; col++ ) {
			Connectivity(row, col) = C[row][col]; 
		}
	}

	// Pass in desired node locations. currently, keeping everything but payload
	// in place
	myVec x = myVec(nodes.size() );
	myVec y = myVec(nodes.size() );
	myVec z = myVec(nodes.size() );
	for( int i = 0; i < cables.size(); i++ ) {
		x(i) = nodes[i]->getX();
		y(i) = nodes[i]->getY();
		z(i) = nodes[i]->getZ();
	}
  // "x" = right positive, zero is middle
  // "y" = vertical positive, zero is ground
  // "z" = away from camera positive, zero is middle
  // add middle to offsets to get absolute location
	x(x.size()-1) = target.getX() + 0; // -1 because zero indexed
	y(y.size()-1) = target.getY() + 8.42;
	z(z.size()-1) = target.getZ() + 0;
  // btVector3 relativeCubeTarget = btVector3(-8.42,0,0);
  // order of nodes:
  //  dist horiz left
  //  dist horiz right
  //  prox horiz left
  //  prox horiz right
  //  top horiz prox
  //  top horiz dist
  //  bott horiz prox
  //  bott horiz dist
  //  right vert bott
  //  right vert top
  //  left vert bott
  //  left vert top
  //  middle

	myVec ConVeX = prod(Connectivity, x); // C*x
	myVec ConVeY = prod(Connectivity, y); // C*y
	myVec ConVeZ = prod(Connectivity, z); // C*z
	// cout<<"connectivity size: " <<Connectivity.size1() << ", " <<Connectivity.size2() <<endl;
	// cout<<"x size: " <<x.size()<<endl;
	// cout<<"ConVeX size: " <<ConVeX.size() <<endl;

	//diagonalized versions of C*x, C*y, C*z 
	myMat diaX = diag(ConVeX);
	myMat diaY = diag(ConVeY);
	myMat diaZ = diag(ConVeZ);

	// C.transpose * diag( C*x ), ...
	myMat temp1 = prod( ublas::trans(Connectivity), diaX ); 
	myMat temp2 = prod( ublas::trans(Connectivity), diaY );
	myMat temp3 = prod( ublas::trans(Connectivity), diaZ );

	// Concatenate the matrices to make matrix A
	myMat A = myMat(temp1.size1()*3, temp1.size2(), true);
	for( int i = 0; i < temp1.size1(); i++ ) {
		ublas::row(A, i) = ublas::row(temp1, i);
		ublas::row(A, i + temp1.size1() ) = ublas::row(temp2, i);
		ublas::row(A, i + 2*temp1.size1() ) = ublas::row(temp3, i);
	}

  cout<< "A: "<<endl;
  for( int i = 0; i < A.size1(); i++ ) {
    cout<< ublas::row(A,i)<<endl;
  }
  cout<<endl;

	// // print A (for debugging)
	// cout<<endl;
	// cout<<endl;
	// cout<<endl;
	// for( int i = 0; i < A.size1(); i++ ) {
	//   cout<< ublas::row(A,i)<<endl;
	// }

	// make an external forces matrix p, which consists only of gravity. the
	// thirteenth value is based on the payload because node 13 is at the
	// payload's center

	myVec p = myVec(A.size1(), false );
	p( p.size() - 1 ) = -GRAVITY*PAYLOAD_MASS;//-10000000000;//-GRAVITY*PAYLOAD_MASS;
  // only modify the z elements. these are the last 3rd of the matrix
	for( int i = p.size() - 2; i >= p.size() - p.size()/3; i-- ) {
		p( i ) = 0;//-GRAVITY*ROD_MASS/2;
		// p( i ) = -GRAVITY*ROD_MASS/2; // uncomment if we want to apply gravity at
    // rod endpoints
	}


  cout<< "going to do armadillo stuff"<<endl;
  // build armadillo matrix so we can use its qr decomp
  mat A_temp = mat(A.size1(), A.size2() );
  for( int row = 0; row < A.size1(); row++ ) {
    for( int col = 0; col < A.size2(); col++ ) {
      A_temp(row, col) = A(row, col);
    }
  }

  mat A_temp_dag = pinv(A_temp);
  cout<< "A_temp size = "<<size(A_temp)<<endl;
  cout<< "A_dag size = "<<size(A_temp_dag)<<endl;

  // Convert from armadillo-friendly matrix to our default matrix format
	myMat A_dag = myMat(A_temp_dag.n_rows, A_temp_dag.n_cols, false);
  for( int row = 0; row < A_temp_dag.n_rows; row++ ) {
    for( int col = 0; col < A_temp_dag.n_cols; col++ ) {
      A_dag(row, col) = A_temp_dag(row, col);
    }
  }
  // cout<< "Q*R - A: " <<prod( Q, R ) - A<<endl<<endl;



  // cout<< "R_inv: "<<endl;
  // for( int i = 0; i < R_inv.size1(); i++ ) {
  //   cout<< ublas::row(R_inv,i)<<endl;
  // }
  // cout<<endl;

  // cout<< "A_dag: "<<endl;
  // for( int i = 0; i < A_dag.size1(); i++ ) {
  //   cout<< ublas::row(A_dag,i)<<endl;
  // }
  // cout<<endl;


  // Lol, everything seems right except that the first column is sign-flipped.
  // this probably means something is pretty wrong, but we're just going to flip
  // it back for now. hahaha!
  // ublas::column(A_dag, 0) = -1 * ublas::column(A_dag, 0);
	// cout<< "A_dag: "<< A_dag<<endl;
	// cout<< "A_dag obtained!!"<<endl;

	// cout<< "A_dag size: "<< A_dag.size1()<< ", " <<A_dag.size2() <<endl;
	// cout<< "A size: " <<A.size1()<< ", " <<A.size2() <<endl;
	// Vtemp is A_dag - prod for all the nodes in the connectivity matrix. V is
	// the same thing, but only for the first s nodes (where s = number of cables)
	myMat Vtemp = ublas::identity_matrix<double>(A_dag.size1()) - prod(A_dag,A);
  // cout<< "here"<<endl;
	myMat V = myMat( NUM_CABLES, Vtemp.size2() );


	for( int row = 0; row < V.size1(); row++ ) {
		for( int col = 0; col < V.size2(); col++ ) {
			V(row, col) = Vtemp(row, col);
		}
	}
	// cout<< "V obtained!!"<<endl;

	myMat A_dag_s = myMat( NUM_CABLES, A_dag.size2() );
	for( int row = 0; row < A_dag_s.size1(); row++ ) {
		for( int col = 0; col < A_dag_s.size2(); col++ ) {
			A_dag_s(row, col) = A_dag(row, col);
		}
	}

  // cout<< "A_dag_s: "<<endl;
  // for( int i = 0; i < A_dag_s.size1(); i++ ) {
  //   cout<< ublas::row(A_dag_s,i)<<endl;
  // }
  // cout<<endl;


  // cout<< "V: "<<endl;
  // for( int i = 0; i < V.size1(); i++ ) {
  //   cout<< ublas::row(V,i)<<endl;
  // }
  // cout<<endl;


  // cout<< "p: "<<endl;
  // cout<< p<<endl;
  // cout<<endl;

	myVec w = findW( V, A_dag_s, p);


  // cout<< "w_initial: "<<w<<endl;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;
  // w(0) = 1;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;
  // w(1) = 1;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;
  // w(2) = 1;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;
  // w(3) = 1;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;
  // w(4) = 1;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;
  // w(5) = 1;
  // cout<< "cost: "<< cost( V, w, A_dag_s, p)<<endl;

	myMat I_sr = myMat( Vtemp.size1(), Vtemp.size1(), false);
	for( int index = 0; index < Vtemp.size1(); index++ ) {
		I_sr(index, index) = 1;
	}

	myVec q = myVec( Vtemp.size1() );
	// cout<< "A_dag size: " << A_dag.size1() << ", " <<A_dag.size2() <<endl;
	// cout<< "p size: " << p.size() <<endl;
	// cout<< "Vsize: " << V.size1() << ", " <<V.size2() <<endl;
	// cout<< "Vtemp size: " << Vtemp.size1() << ", " <<Vtemp.size2() <<endl;
	// cout<< "q size: " << q.size() <<endl;
	myVec first_half = prod(A_dag, p);
  cout<<"naive force densities:"<<first_half<<endl;

  for( int i = 0; i < NUM_CABLES+1; i++ ) {
    cout<<"node "<< i <<" location: "<< *(nodes[i]) <<endl;
  }

	myVec sec_half = prod( I_sr - prod(A_dag, A), w);
	// cout<< "first half of (8) " <<first_half<<endl;
	// cout<< "second half of (8) " <<sec_half <<endl;
	q = prod(A_dag, p) + prod( I_sr - prod(A_dag, A), w);
  for( int i = 0; i < q.size() ; i++ ) {
    q(i) = round(q(i));
  }
  // cout<< endl<<"q:"<<q<<endl<<endl;
	// cout<< q<<endl;
	// cout<< "A*q - p: " << prod(A, q) - p<<endl<<endl;;
	// cout<< "p: " << p <<endl;


  // TODO: subtract correction from passed in targetLengths
	// cout<<"RLength correction: " << element_prod(targetActualLengths, q)/springConstant <<endl;
	// for( int i = 0; i < cables.size(); i++) {
	//   // if you want to move stuff without inverse kinematics, subtract five
	//   // from the target lengths to pretension cables
	//   cout<<"Rlength: "<<targetRLengths[i] << endl;
	//   //cout<< cables[i]->toString() <<endl;
	// }



	return;
	// take a position target
	//
	// Calculate forces in the cables to place node 13 at target
	//

	// 2. write out p matrix (external forces)

	// 3. transform C to A

	// 4. minimize thing, ask jeff about this
}



btVector3 BallDropperYAML::getSuperballCenter() {
  btVector3 center = btVector3();
  for( int i = 0; i < cables.size(); i++ ) {
    center += (*(nodes[i]))/( (double) cables.size() );
  }
  return center;
}



myVec BallDropperYAML::findW(myMat V, myMat A_dag_s, myVec forces ) {
  // cout<< "V: "<<endl;
  // for( int i = 0; i < V.size1(); i++ ) {
  //   cout<< ublas::row(V,i)<<endl;
  // }
  // cout<<endl;
  //
  // cout<< "A_dag_s: "<<endl;
  // for( int i = 0; i < A_dag_s.size1(); i++ ) {
  //   cout<< ublas::row(A_dag_s,i)<<endl;
  // }
  // cout<<endl;
  //
  // cout<< "forces: "<<endl;
  // cout<<forces<<endl;
  // cout<<endl;

	Matrix<double> G, CE, CI;
	Vector<double> g0, ce0, ci0, x;
	int n, m, p;
	double sum = 0.0;
	char ch;

	myMat temp = prod(trans(V),V);
	n = temp.size1();
	m = temp.size2();
	G.resize(n, m);
	{
		for (int row = 0; row < n; row++)	{
			for (int col = 0; col < m; col++) {
				G[row][col] = temp(row, col);
			}
		}
	}

	// cout<< " Sizes" <<endl;
	// cout<< " Vtrans: " <<trans(V).size1() << ", " << trans(V).size2()<< endl;
	// cout<< " A_dag_s: " <<A_dag_s.size1() << ", " << A_dag_s.size2()<< endl;
	// cout<< " p: " <<forces.size()<<endl;
	// cout<< " Vtrans: " <<trans(V).size1() << ", " << trans(V).size2()<< endl;



	temp = prod( trans(V), A_dag_s);
	myVec tempVec = prod( temp, forces );
	n = tempVec.size();
	g0.resize( n );
	{
		for (int row = 0; row < n; row++) {
			g0[row] = tempVec(row);
		}
	}

	n = 12;
	m = 0;
	CE.resize(n, m);
	// CE = {};

	// {
	// 	for (int i = 0; i < n; i++){
	// 		for (int j = 0; j < m; j++){
	//       CE[i][j] = 0;
	//     }
	//   }
	// } 

	ce0.resize(0);
	// ce0 = {};
	// ce0[0] = 0;

	// p = 3;
	temp = trans(V);
	n = temp.size1();
	m = temp.size2();
	CI.resize(n, m);
	{
		for (int row = 0; row < n; row++) {
			for (int col = 0; col < m; col++) {
				CI[row][col] = temp(row, col);
			}
		}
	}


	tempVec = prod( A_dag_s, forces);
	n = tempVec.size();
	ci0.resize(n);
	{
		for (int row = 0; row < n; row++) {
			ci0[row] = tempVec(row);
		}
	}

	x.resize(n);

	std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
	// std::cout << "f: " << solve_quadprog(G, g0, CI, ci0, x) << std::endl;
	std::cout << "x: " << x << std::endl;
	// Debugging
	// for (int i = 0; i < n; i++)
	//   std::cout << x[i] << ' ';
	// std::cout << std::endl;

	// FOR DOUBLE CHECKING COST since in the solve_quadprog routine the matrix G is modified 

	temp = prod(trans(V),V);
	n = temp.size1();
	m = temp.size2();
	G.resize(n, m);
	{
		for (int row = 0; row < n; row++)	{
			for (int col = 0; col < m; col++) {
				G[row][col] = temp(row, col);
			}
		}
	}
	// {
	//   std::istringstream is("4, -2,"
	// 												"-2, 4 ");
	//
	// 	for (int i = 0; i < n; i++)
	// 		for (int j = 0; j < n; j++)
	// 			is >> G[i][j] >> ch;
	// }

	// cout << "Double checking cost: ";
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			sum += x[i] * G[i][j] * x[j];
	sum *= 0.5;	

	for (int i = 0; i < n; i++)
		sum += g0[i] * x[i];
	// std::cout << sum << std::endl;

	// once we figure out what w actually is, resolve confusion between x and w
	myVec w = myVec(x.size());
	for( int i = 0; i < x.size(); i++ ) {
		w(i) = x[i];
	}
	return w;
}


/**
*/
//BallDropperYAML::rotate( btVector3* pt, btVector3* euler angles)




/**
 * Update the necessary RLengths so that the cube is pulled towards the set
 * point
 * @input
 */
void BallDropperYAML::updateRLengths( btVector3 relativeCubeTarget ) {
  cout<< "updating RLengths"<<endl;
  cout<<"target: "<<relativeCubeTarget<<endl;

  /////////// using the center as one node /////////
	for( int i = 0; i < cables.size(); i++) {
		targetLengths(i) = (*(nodes[i]) - relativeCubeTarget).norm();
		desiredCableDirections.push_back( relativeCubeTarget - *(nodes[i]) );
    desiredCableDirections[i].normalize();
		//cout<< cables[i]->toString() <<endl;
    targetRLengths(i) = targetLengths(i);// - 4.5;
    // if( targetRLengths(i) < MIN_RL ) {
    //   targetRLengths(i) = MIN_RL;
    // }
    // cout<<" targetLength1 : "<<
    cout<< "uncorrected RLength "<<i<<": "<< targetRLengths(i)<<endl;
	}
  

  // // use if we want to correct for orientation (START)
  // ///////// Using the actual edge connection points ////////
  // // This doesn't work very well. If the cube is close to the target point then
  // // it's good, but otherwise, the cable lengths are really weird. Perhaps what 
  // // we should use is relative position (within the superball structure)
  // // find the orientation of the cube
  // btVector3 eulerAngles = cube[0]->orientation();
  // btMatrix3x3 rotMatrix = btMatrix3x3();
  // btQuaternion quatOrientation;
  // rotMatrix.setEulerYPR(eulerAngles.getX(), eulerAngles.getY(),
  //     eulerAngles.getZ() ); // fill rotMatrix
  // // set quatOrientation as the quaternion representing the body's orientation
  // rotMatrix.getRotation( quatOrientation ); 
  //
  // #<{(| get the absolute coordinates of the edge points |)}>#
  // currDisplacementVectors.clear();
  // updateNodes();
  // for( int i = 0; i < cables.size(); i++ ) {
  //   // set currDisplacementVectors to  initDisplacement, then rotate them
  //   currDisplacementVectors.push_back( initDisplacementVectors[i] );
  //   quatRotate( quatOrientation , currDisplacementVectors[i] );
  //   // add displacement to target to get edge point (assuming rotation changes
  //   // little. with iteration, this is pretty much p feedback
  //   btVector3 currEdge = *middle + currDisplacementVectors[i];
  //   targetLengths(i) = ( *(nodes[i]) - currEdge ).norm();
  //   cout<< "corrected target Length "<< i << ": "<<targetLengths(i) <<endl;
  // }
  // // for( int j = 0; j < nodes.size() - 1; j++ ) {
  // //   int closest = -1;
  // //   int distance = 1000000;
  // //   for( int i = 0; i < currDisplacementVectors.size(); i++ ) {
  // //     double distanceToEdgeI = ( relativeCubeTarget- *(nodes[j]) + currDisplacementVectors[i]).norm();
  // //     if( distanceToEdgeI < distance ) {
  // //       closest = i;
  // //       distance = distanceToEdgeI;
  // //     }
  // //   }
  // //   cout<< "closest edge to rod end "<< j <<" is edge "<<closest <<endl;
  // // }
  // for( int i = 0; i < cables.size(); i++ ) {
  //   targetRLengths(i) = targetLengths(i) - 0;
  // }  
  // // Use if we want to correct for orientation (END)

  // cout<< "target: "<< target<<endl;
  // cout<< "updateRLengths thinks node 1 is at: "<< *nodes[1]<<endl;
  // cout<< "distance from target to node: "<< nodes[1]->distance(target)<<endl;
  // cout<< "desired cable direction 1 : "<< desiredCableDirections[1]<<endl;
  // // cout<< "target: "<< target<<endl;
  // cout<< "updateRLengths thinks the cube will be at: "<< *nodes[1] + targetLengths(1)*desiredCableDirections[1] <<endl;
  cout<< "finished updating RLengths"<<endl;
  // return targetRLengths;
}


void BallDropperYAML::updateNodes() {
 nodes.erase(nodes.begin(), nodes.end() );
  //for each rod in the RODS, add its end points to the nodes vector
  for( size_t i = 0; i < rods.size(); i++ ) {
    pair< btVector3*, btVector3* > endPoints = rods[i]->ends();
    nodes.insert( nodes.begin(), endPoints.first );
    nodes.insert( nodes.begin(), endPoints.second );
  }
  // order of nodes:
  //  dist horiz right
  //  dist horiz left
  //  prox horiz right
  //  prox horiz left
  //  top horiz dist
  //  top horiz prox
  //  bott horiz dist
  //  bott horiz prox
  //  right vert top
  //  right ver bott
  //  left vert top
  //  left vert bott
  //  middle

  //add the middle of the cube to the nodes vector
  btVector3 top = *(cube[0]->ends().first);
  btVector3 bottom = *(cube[0]->ends().second);
  *middle = (top + bottom)/2;
  nodes.insert( nodes.end(), middle ); // why add middle to nodes? because we use it in connectivity matrix
  // for( int i = 0; i < nodes.size(); i++ ) {
  //   // cout<< *nodes[i] << ", ";
  // }
  // cout<<endl;
  return;
}



/* old playing around-thing, tightens some cables and lengthens others.
 * maintains a min tension as well
 */
void BallDropperYAML::onStep(TensegrityModel& subject, double dt)
{
	double nextRestLength;
	double currRestLength;
	double minRestLength;
	timePassed += dt;
  btVector3 relativeCubeTarget = btVector3(0,8,0);


  if( timePassed < 2*dt){



  updateRLengths( relativeCubeTarget ); // updates targetRLengths
  inverseKin( targetRLengths, C, CROWS, CCOLS, springConstant, nodes, relativeCubeTarget );
  return;

	// if( timePassed < 2*dt ) {
	//   cout<< "current lengths: ";
	//   for( int i = 0; i < NUM_CABLES; i ++ ) {
	//     cout<< cables[i]->getCurrentLength() << ", ";
	//   }
	//   cout<< endl;
	//   cout<< "rest lengths: ";
	//   for( int i = 0; i < NUM_CABLES; i ++ ) {
	//     cout<< cables[i]->getRestLength() << ", ";
	//   }
	//     cout<<endl;
	// }
	if( timePassed > startTime*2.0/7.0 ) {
    // cout<< "targetLengths: "<<targetRLengths<<endl;
		for( int i = 0; i < NUM_CABLES; i ++ ) {
			cables[i]->setControlInput(targetRLengths(i), 0.001*dt);
		}
	}
  if( (int) (timePassed/dt) % 10000 == 0 ) {
    // cout<<"here"<<endl;
    // cout<< "middle:"<<(*nodes.back())<<endl;
    // cout<< "updateRLengths thinks node 1 is at: "<< *nodes[1]<<endl;
    // cout<< endl<<"rLengths called: "<<targetRLengths<<endl<<endl;
  }
	// if( timePassed > startTime ) {
	//   // if enough time has passed, actuate cables
	//   for (size_t i = 0; i < shorten_vector.size(); i ++) {	
	//     // shorten the cables in shorten_vector
	//     currRestLength = shorten_vector[i]->getRestLength();
	//     // Calculate the minimum rest length for this cable.
	//     // Remember that minLength is a percent.
	//     minRestLength = initialRL[shorten_vector[i]->getTags()] * minLength;
	//     // If the current rest length is not too small
	//     if( currRestLength > minRestLength ) {
	//       // output a progress bar for the controller, to track when control occurs.
	//       cout << ".";
	//       nextRestLength = currRestLength - rate * dt;
	//       //DEBUGGING
	//       //cout << "Next Rest Length: " << nextRestLength << endl;
	//       shorten_vector[i]->setControlInput(nextRestLength,dt);
	//     }
	//   }   
	//   for (size_t i = 0; i < lengthen_vector.size(); i ++) {	
	//     // lengthen the cables in lengthen_vector
	//     currRestLength = lengthen_vector[i]->getRestLength();
	//     // Calculate the minimum rest length for this cable.
	//     // Remember that minLength is a percent.
	//     minRestLength = initialRL[lengthen_vector[i]->getTags()] * minLength;
	//     // If the current rest length is not too small
	//     if( lengthen_vector[i]->getTension() > 1 ){
	//       // output a progress bar for the controller, to track when control occurs.
	//       cout << "lengthen";
	//       nextRestLength = currRestLength + rate * dt;
	//       //DEBUGGING
	//       //cout << "Next Rest Length: " << nextRestLength << endl;
	//       lengthen_vector[i]->setControlInput(nextRestLength,dt);
	//     }
	//     else if( currRestLength > minRestLength) {
	//       cout << "shorten";
	//       nextRestLength = currRestLength - rate * dt;
	//       //DEBUGGING
	//       //cout << "Next Rest Length: " << nextRestLength << endl;
	//       lengthen_vector[i]->setControlInput(nextRestLength,dt);
	//     }
	//
	//   }   
	// }
  }
}



/* ////////////// Cost Minimization /////////// */

double BallDropperYAML::cost(myMat V, myVec w, myMat A_dag_s, myVec p ) {
  if( ( prod(A_dag_s, p) + prod( V, w ) )(0) < 0 ) {
    cout<< endl<< "Inputted matrices do not meet constraints! (w = " <<w <<")."<<endl<<endl;
  }
  // double cost = prod( prod( trans( w ), trans( V ) ), prod( V, w ) ) +  
  //   2 * prod( prod( trans( w ), trans( V ) ), prod( A_dag_s, p ) );

  myVec first_two =  prod( w, trans( V ) );
  myVec sec_two = prod( V, w );
  myVec sec_first_two = prod( w, trans( V ) );
  myVec sec_sec_two = prod( A_dag_s, p );
  double cost = inner_prod( first_two, sec_two ) + 2 * inner_prod( sec_first_two, sec_sec_two );

  return cost;
}





/* ///////////// Matrix Operations /////////// */


/* make a diagonal matrix out of a vector
*/
myMat BallDropperYAML::diag( myVec inVec ) {

	myMat diagonal = myMat( inVec.size(), inVec.size() );

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
myVec BallDropperYAML::proj(myVec a, myVec e) { // projection of vector A onto e
	myVec result = e*( inner_prod(e, a) / (inner_prod(e, e) ) );
	return result;
}


pair<myMat, myMat> BallDropperYAML::grammyDecomp( myMat A ) {
	cout<< "intput to decomp: " << A <<endl;
	myMat U = myMat(A.size1(), A.size2() );
	myMat E = myMat(A.size1(), A.size2() );

	// cout<< "loops ends when col = " << A.size2()
	for( int col = 0; col < A.size2(); col++ ) {

		for( int j = 0; j < col; j++ ) {
			// cout<< "j is "<<j<< endl;
			// cout<< "col is "<<col<< endl;
			ublas::column(U, col) -= proj( ublas::column(A, col), ublas::column(U, j));
			// cout<< "proj works"<<endl;
		}

		ublas::column(E, col) = ublas::column(U, col) / norm_2( ublas::column(U, col) );
	}
	myMat Q = E;
	myMat R = prod(ublas::trans(Q),A);
  for( int index = 0; index < R.size1(); index++ ) {
    assert( index < R.size2());
    assert(R(index, index) != 0);
    // if( R(index,index) == 0 ) {
    //   cout<< "Oh no! You LOSER! R is not invertible!!!"<<endl;
    // }
  }
	// cout<< "U: " <<U << endl;
	// cout<< "Q: " <<Q << endl;
  cout<<endl;
	cout<< "R: " <<R << endl;
  cout<<endl;
  cout<< "finished gramm-Schmidtt decomp"<<endl;
	return pair<myMat, myMat>(Q, R);
}


bool BallDropperYAML::InvertMatrix(const myMat* input, myMat* inverse)
{
	typedef ublas::permutation_matrix<std::size_t> pmatrix;

	// create a working copy of the input
	myMat A(*input);

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


