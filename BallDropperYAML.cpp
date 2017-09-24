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
    cout<< "node " <<i <<": " <<*(initialNodes[i])<<endl;
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


void BallDropperYAML::initializeController() {


	/* Testing functions */
	// Grammy Decomp
	// mat A_test0 = mat(3,3);
	// A_test0(0,0) = 1;
	// A_test0(0,1) = 1;
	// A_test0(0,2) = 0;
	// A_test0(1,0) = 1;
	// A_test0(1,1) = 0;
	// A_test0(1,2) = 1;
	// A_test0(2,0) = 0;
	// A_test0(2,1) = 1;
	// A_test0(2,2) = 1;
	// cout<< "A_test0: " <<A_test0 <<endl;
	//
	// pair<mat, mat > output0 = grammyDecomp(A_test0);
	// mat Q_test0 = output0.first;
	// mat R_test0 = output0.second;
	// cout<< "Q_test0: " <<Q_test0 <<endl;
	// cout<< "R_test0: " <<R_test0 <<endl;


	// mat A_test1 = mat(4,3);
	// A_test1(0,0) = 1;
	// A_test1(0,1) = -1;
	// A_test1(0,2) = 4;
	// A_test1(1,0) = 1;
	// A_test1(1,1) = 4;
	// A_test1(1,2) = -2;
	// A_test1(2,0) = 1;
	// A_test1(2,1) = 4;
	// A_test1(2,2) = 2;
	// A_test1(3,0) = 1;
	// A_test1(3,1) = -1;
	// A_test1(3,2) = 0;
	// cout<< "A_test1: " <<A_test1 <<endl;
	//
	// pair<mat, mat > output1 = grammyDecomp(A_test1);
	// mat Q_test1 = output1.first;
	// mat R_test1 = output1.second;
	// cout<< "Q_test1: " <<Q_test1 <<endl;
	// cout<< "R_test1: " <<R_test1 <<endl;



	// mat A_test2 = mat(4,4);
	// // A_test2(0,0) = 1;
	// // A_test2(0,1) = 2;
	// // A_test2(0,2) = 3;
	// // A_test2(0,3) = 4;
	// // A_test2(1,0) = 1;
	// // A_test2(1,1) = 2;
	// // A_test2(1,2) = 3;
	// // A_test2(1,3) = 4;
	// // A_test2(2,0) = 1;
	// // A_test2(2,1) = 2;
	// // A_test2(2,2) = 3;
	// // A_test2(2,3) = 4;
	// // A_test2(3,0) = 1;
	// // A_test2(3,1) = 2;
	// // A_test2(3,2) = 3;
	// // A_test2(3,3) = 4;
	// A_test2(0,0) = 3;
	// A_test2(0,1) = 0;
	// A_test2(0,2) = 0;
	// A_test2(0,3) = 0;
	// A_test2(1,0) = 0;
	// A_test2(1,1) = 2;
	// A_test2(1,2) = 0;
	// A_test2(1,3) = 0;
	// A_test2(2,0) = 0;
	// A_test2(2,1) = 0;
	// A_test2(2,2) = -3;
	// A_test2(2,3) = 0;
	// A_test2(3,0) = 0;
	// A_test2(3,1) = 0;
	// A_test2(3,2) = 0;
	// A_test2(3,3) = 1;
	// cout<< "A_test2: " <<A_test2 <<endl;
  //
	// mat inverse = mat(4,4);
	// (void) InvertMatrix(&A_test2, &inverse);
	// cout<< "inverse: " <<inverse <<endl;











	double distance;
	// btVector3 target = btVector3( btScalar(4), btScalar(8.42), btScalar(0) );
  //     move       alllll      Assign location here
	//          (towards ground) (camera right) (towards camera)       
  // middle = (0. 8.42, 0);
	target = btVector3( btScalar(-4), btScalar(8.42), btScalar(0) );
  updateRLengths();

	Connectivity = ublas::matrix<double>(12, 13);
	for( int row = 0; row < 12; row++ ) {
		for( int col = 0; col < 13; col++ ) {
			Connectivity(row, col) = C[row][col]; 
		}
	}

	//   cout<< Connectivity(1,1) <<", " <<endl;
	//   cout<< Connectivity(1,2) <<", " <<endl;
	//   cout<< Connectivity(1,3) <<", " <<endl;
	//   cout<< Connectivity(1,4) <<", " <<endl;


	// ublas::vector<double> x = ublas::vector<double>(nodes.size() );
	// ublas::vector<double> y = ublas::vector<double>(nodes.size() );
	// ublas::vector<double> z = ublas::vector<double>(nodes.size() );
	// Pass in desired node locations. currently, keeping everything but payload
	// in place
	vec x = vec(nodes.size() );
	vec y = vec(nodes.size() );
	vec z = vec(nodes.size() );
	for( int i = 0; i < cables.size(); i++ ) {
		x(i) = nodes[i]->getX();
		y(i) = nodes[i]->getY();
		z(i) = nodes[i]->getZ();
	}
	x(x.size()-1) = target.getX();
	y(y.size()-1) = target.getY();
	z(z.size()-1) = target.getZ();

	vec ConVeX = prod(Connectivity, x); // C*x
	vec ConVeY = prod(Connectivity, y); // C*y
	vec ConVeZ = prod(Connectivity, z); // C*z
	cout<<"connectivity size: " <<Connectivity.size1() << ", " <<Connectivity.size2() <<endl;
	// cout<<"x size: " <<x.size()<<endl;
	// cout<<"ConVeX size: " <<ConVeX.size() <<endl;

	//diagonalized versions of C*x, C*y, C*z 
	mat diaX = diag(ConVeX);
	mat diaY = diag(ConVeY);
	mat diaZ = diag(ConVeZ);

	// C.transpose * diag( C*x ), ...
	mat temp1 = prod( ublas::trans(Connectivity), diaX ); 
	mat temp2 = prod( ublas::trans(Connectivity), diaY );
	mat temp3 = prod( ublas::trans(Connectivity), diaZ );

	// Concatenate the matrices to make matrix A
	mat A = mat(temp1.size1()*3, temp1.size2(), true);
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

	// make a external forces matrix p, which consists only of gravity. the
	// thirteenth value is based on the payload because node 13 is at the
	// payload's center

	vec p = vec(A.size1(), false );
	for( int i = p.size() - 2; i >= p.size() - p.size()/3; i-- ) {
		p( i ) = -GRAVITY*ROD_MASS/2;
	}
	p( p.size() - 1 ) = -GRAVITY*PAYLOAD_MASS;


	pair<mat, mat> QRmats = grammyDecomp(A);
	cout<< "grammyDecomp is working"<<endl;
	mat Q = QRmats.first;
	mat R = QRmats.second;
	mat R_inv = mat(R.size1(),R.size2());
	mat A_dag;

  cout<< "R: "<<endl;
  for( int i = 0; i < R.size1(); i++ ) {
    cout<< ublas::row(R,i)<<endl;
  }
  cout<<endl;


  cout<< "Q_trans: "<<endl;
  for( int i = 0; i < trans(Q).size1(); i++ ) {
    cout<< ublas::row( trans(Q) ,i)<<endl;
  }
  cout<<endl;


	if(  InvertMatrix( &R, &R_inv ) ) {
		A_dag = prod( R_inv, trans(Q)) ;
	}
	else {
		cout<< "R-inversion failed... SAD!"<<endl;
	}

  // cout<< "Q*R - A: " <<prod( Q, R ) - A<<endl<<endl;

  cout<< "R_inv: "<<endl;
  for( int i = 0; i < R_inv.size1(); i++ ) {
    cout<< ublas::row(R_inv,i)<<endl;
  }
  cout<<endl;

  cout<< "A_dag: "<<endl;
  for( int i = 0; i < A_dag.size1(); i++ ) {
    cout<< ublas::row(A_dag,i)<<endl;
  }
  cout<<endl;


  // Lol, everything seems right except that the first column is sign-flipped.
  // this probably means something is pretty wrong, but we're just going to flip
  // it back for now. hahaha!
  // ublas::column(A_dag, 0) = -1 * ublas::column(A_dag, 0);
	// cout<< "A_dag: "<< A_dag<<endl;
	cout<< "A_dag obtained!!"<<endl;


	cout<< "A_dag size: "<< A_dag.size1()<< ", " <<A_dag.size2() <<endl;
	cout<< "A size: " <<A.size1()<< ", " <<A.size2() <<endl;
	// Vtemp is A_dag - prod for all the nodes in the connectivity matrix. V is
	// the same thing, but only for the first s nodes (where s = number of cables)
	mat Vtemp = ublas::identity_matrix<double>(A_dag.size1()) - prod(A_dag,A);
	mat V = mat( NUM_CABLES, Vtemp.size2() );


	for( int row = 0; row < V.size1(); row++ ) {
		for( int col = 0; col < V.size2(); col++ ) {
			V(row, col) = Vtemp(row, col);
		}
	}
	cout<< "V obtained!!"<<endl;

	mat A_dag_s = mat( NUM_CABLES, A_dag.size2() );
	for( int row = 0; row < A_dag_s.size1(); row++ ) {
		for( int col = 0; col < A_dag_s.size2(); col++ ) {
			A_dag_s(row, col) = A_dag(row, col);
		}
	}


  cout<< "V: "<<endl;
  for( int i = 0; i < V.size1(); i++ ) {
    cout<< ublas::row(V,i)<<endl;
  }
  cout<<endl;

  cout<< "A_dag_s: "<<endl;
  for( int i = 0; i < A_dag_s.size1(); i++ ) {
    cout<< ublas::row(A_dag_s,i)<<endl;
  }
  cout<<endl;

  cout<< "p: "<<endl;
  cout<< p<<endl;
  cout<<endl;

	vec w = findW( V, A_dag_s, p);


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



	mat I_sr = mat( Vtemp.size1(), Vtemp.size1(), false);
	for( int index = 0; index < Vtemp.size1(); index++ ) {
		I_sr(index, index) = 1;
	}

	vec q = vec( Vtemp.size1() );
	// cout<< "A_dag size: " << A_dag.size1() << ", " <<A_dag.size2() <<endl;
	cout<< "p size: " << p.size() <<endl;
	cout<< "Vsize: " << V.size1() << ", " <<V.size2() <<endl;
	// cout<< "Vtemp size: " << Vtemp.size1() << ", " <<Vtemp.size2() <<endl;
	cout<< "q size: " << q.size() <<endl;
	vec first_half = prod(A_dag, p);
	vec sec_half = prod( I_sr - prod(A_dag, A), w);
	// cout<< "first half of (8) " <<first_half<<endl;
	// cout<< "second half of (8) " <<sec_half <<endl;
	q = prod(A_dag, p) + prod( I_sr - prod(A_dag, A), w);
  for( int i = 0; i < q.size() ; i++ ) {
    q(i) = round(q(i));
  }
  cout<< endl<<"q:"<<q<<endl<<endl;
	// cout<< q<<endl;
	cout<< "A*q - p: " << prod(A, q) - p<<endl<<endl;;
	// cout<< "p: " << p <<endl;


	cout<<"RLength correction: " << element_prod(targetLengths, q)/springConstant <<endl;
	for( int i = 0; i < cables.size(); i++) {
	  // if you want to move stuff without inverse kinematics, subtract five
	  // from the target lengths to pretension cables
	  cout<<"Rlength: "<<targetRLengths[i] << endl;
	  //cout<< cables[i]->toString() <<endl;
	}



	return;
	// take a position target
	//
	// Calculate forces in the cables to place node 13 at target
	//

	// 2. write out p matrix (external forces)

	// 3. transform C to A

	// 4. minimize thing, ask jeff about this
}

vec BallDropperYAML::findW(mat V, mat A_dag_s, vec forces ) {
  cout<< "V: "<<endl;
  for( int i = 0; i < V.size1(); i++ ) {
    cout<< ublas::row(V,i)<<endl;
  }
  cout<<endl;

  cout<< "A_dag_s: "<<endl;
  for( int i = 0; i < A_dag_s.size1(); i++ ) {
    cout<< ublas::row(A_dag_s,i)<<endl;
  }
  cout<<endl;

  cout<< "forces: "<<endl;
  cout<<forces<<endl;
  cout<<endl;

	Matrix<double> G, CE, CI;
	Vector<double> g0, ce0, ci0, x;
	int n, m, p;
	double sum = 0.0;
	char ch;

	mat temp = prod(trans(V),V);
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
	vec tempVec = prod( temp, forces );
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
	vec w = vec(x.size());
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
 */
void BallDropperYAML::updateRLengths() {
	for( int i = 0; i < cables.size(); i++) {
		// if you want to move stuff without inverse kinematics, subtract 4.5
		// from the target lengths to pretension cables
		targetLengths(i) = (nodes[i]->distance(target));
		desiredCableDirections.push_back( target - *(nodes[i]) );
    desiredCableDirections[i].normalize();
		//cout<< cables[i]->toString() <<endl;
    targetRLengths(i) = targetLengths(i) - 4.5;
    // if( targetRLengths(i) < MIN_RL ) {
    //   targetRLengths(i) = MIN_RL;
    // }
    // cout<<" targetLength1 : "<<
	}
  updateNodes();

  // cout<< "target: "<< target<<endl;
  // cout<< "updateRLengths thinks node 1 is at: "<< *nodes[1]<<endl;
  // cout<< "distance from target to node: "<< nodes[1]->distance(target)<<endl;
  // cout<< "desired cable direction 1 : "<< desiredCableDirections[1]<<endl;
  // // cout<< "target: "<< target<<endl;
  // cout<< "updateRLengths thinks the cube will be at: "<< *nodes[1] + targetLengths(1)*desiredCableDirections[1] <<endl;
}


void BallDropperYAML::updateNodes() {
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
  *middle = (top + bottom)/2;
  nodes.insert( nodes.end(), middle );
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
  if( timePassed == 0.0 ) {
    updateRLengths();
  }
	timePassed += dt;
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
    cout<< endl<<"rLengths called: "<<targetRLengths<<endl<<endl;
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



/* ////////////// Cost Minimization /////////// */

double BallDropperYAML::cost(mat V, vec w, mat A_dag_s, vec p ) {
  if( ( prod(A_dag_s, p) + prod( V, w ) )(0) < 0 ) {
    cout<< endl<< "Inputted matrices do not meet constraints! (w = " <<w <<")."<<endl<<endl;
  }
  // double cost = prod( prod( trans( w ), trans( V ) ), prod( V, w ) ) +  
  //   2 * prod( prod( trans( w ), trans( V ) ), prod( A_dag_s, p ) );

  vec first_two =  prod( w, trans( V ) );
  vec sec_two = prod( V, w );
  vec sec_first_two = prod( w, trans( V ) );
  vec sec_sec_two = prod( A_dag_s, p );
  double cost = inner_prod( first_two, sec_two ) + 2 * inner_prod( sec_first_two, sec_sec_two );

  return cost;
}





/* ///////////// Matrix Operations /////////// */


/* make a diagonal matrix out of a vector
*/
mat BallDropperYAML::diag( vec inVec ) {

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
vec BallDropperYAML::proj(vec a, vec e) { // projection of vector A onto e
	vec result = e*( inner_prod(e, a) / (inner_prod(e, e) ) );
	return result;
}


pair<mat, mat> BallDropperYAML::grammyDecomp( mat A ) {
	// cout<< "intput to decomp: " << A <<endl;
	mat U = mat(A.size1(), A.size2() );
	mat E = mat(A.size1(), A.size2() );

	// cout<< "loops ends when col = " << A.size2()
	for( int col = 0; col < A.size2(); col++ ) {
		// cout<< "col is "<<col<<endl;
		ublas::column(U, col) = ublas::column(A, col);
		for( int j = 0; j < col; j++ ) {
			// cout<< "j is "<<j<< endl;
			// cout<< "col is "<<col<< endl;
			ublas::column(U, col) -= proj( ublas::column(A, col), ublas::column(U, j));
			// cout<< "proj works"<<endl;
		}

		ublas::column(E, col) = ublas::column(U, col) / norm_2( ublas::column(U, col) );
	}
	mat Q = E;
	mat R = prod(ublas::trans(Q),A);
  for( int index = 0; index < R.size1(); index++ ) {
    if( R(index,index) == 0 ) {
      cout<< "Oh no! You LOSER! R is not invertible!!!"<<endl;
    }
  }
	// cout<< "U: " <<U << endl;
	// cout<< "Q: " <<Q << endl;
	// cout<< "R: " <<R << endl;
	return pair<mat, mat>(Q, R);
}


bool BallDropperYAML::InvertMatrix(const mat* input, mat* inverse)
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


