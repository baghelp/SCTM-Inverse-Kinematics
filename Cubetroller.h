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
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */

#ifndef HORIZONTAL_SPINE_CONTROLLER_H
#define HORIZONTAL_SPINE_CONTROLLER_H

/**
 * @file HorizontalSpineController.h -- McRandy trying machine learning
 * @brief Contains the definition of class Cubetroller
 * @author rew Sabelhaus
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

//#include "sensors/tgataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "helpers/FileHelpers.h"
#include <stdexcept>
#include <string.h>
#include <math.h>
#include "./../../QuadProgpp/src/QuadProg++.hh" 


#define GRAVITY 9.8
#define PAYLOAD_MASS 10
#define ROD_MASS 6
#define NUM_CABLES 12
#define MIN_RL 0
#define START_TIME 5.0
#define THETA0_FILE "Theta0.txt"
#define THETA1_FILE "Theta1.txt"
#define THETA2_FILE "Theta2.txt"
#define THETA3_FILE "Theta3.txt"
#define THETA4_FILE "Theta4.txt"
#define THETA5_FILE "Theta5.txt"
#define THETA6_FILE "Theta6.txt"
#define INTERCEPT0_FILE "b0.txt"
#define INTERCEPT1_FILE "b1.txt"
#define INTERCEPT2_FILE "b2.txt"
#define INTERCEPT3_FILE "b3.txt"
#define INTERCEPT4_FILE "b4.txt"
#define INTERCEPT5_FILE "b5.txt"
#define INTERCEPT6_FILE "b6.txt"
#define RL_STEP_SIZE 0.1
#define MIN_TENSION 0.5
#define CENTR_RL 5.27272
using namespace std;
using namespace boost::numeric;

// Forward declarations
class TensegrityModel;
class tgBasicActuator;
typedef ublas::matrix<double> mat;
typedef ublas::vector<double> vec;

/**
 * A controller to apply the length change in the cables of the 3-bar example
 * model, for the NTRT Introduction Seminar on 2016-09-28 in BEST.
 */
class Cubetroller : public tgObserver<TensegrityModel>, public tgSubject<Cubetroller>
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
    Cubetroller( double minTension, double rate,
        double angle);

    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~Cubetroller() { 
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
    pair<btVector3*, btVector3*> getEnds(typeof(tgBaseRigid)* member);

    /**
    */
    // pair<mat*, mat*> QRecomp( mat A );

    /**
    */
    vec proj( vec, vec );

    /**
    */
    pair<mat, mat> grammyDecomp( mat );

    /**
    */
    bool InvertMatrix(const mat* input, mat* inverse);

    /**
    */
    void findW(mat V, mat A_dag, mat forces);

    /**
    */
    void readMatFiles();


    /**
    */
    btVector3 getModelCOM();


    /**
    */
    void readTheta( mat& , string thetaFile );

    /**
    */
    void readIntercept( vec& intercept, string thetaFile );

    /**
    */
    vec ReLU( vec );


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
    mat diag( vec inVec );

    void updateRLengths( );
    
    void updateNodes();


  private:

    /**
     * The private variables for each of the values passed in to the constructor.
     */
    double startTime;
    int step;
    double minTension;
    double rate;
    double angleOfTravel;
    std::vector<tgBasicActuator*> cables;
    std::vector<tgRod*> rods;
    std::vector<tgBox*> cube;
    std::vector< btVector3* > nodes;
    std::string const CABLES[NUM_CABLES] = {"cube_string_1", "cube_string_2",
      "cube_string_3", "cube_string_4", "cube_string_5", "cube_string_6", 
      "cube_string_7", "cube_string_8", "cube_string_9", "cube_string_10", 
      "cube_string_11", "cube_string_12"};
    std::string const RODS[6] = {"rod_1", "rod_2", "rod_3", "rod_4", "rod_5",
      "rod_6"};
    std::string const CUBE = "super_cube";
    // /* Linear Regression on 1 face */
    // double thetarray[4][12] = {
    //   {5.435372  , 4.049139   ,4.657015   ,4.763169   ,4.298920   ,4.989401   ,4.845110   ,4.527898   ,4.797078   ,4.763935   ,4.857165   ,4.567432},
    //   {0.237722  ,-0.100865   ,0.064874  ,-0.122059  ,-0.581762   ,0.588158   ,0.103260  ,-0.152758   ,0.037579  ,-0.112457  ,-0.801214   ,0.682775},
    //   {-0.635718   ,1.281960   ,0.807030   ,0.350293   ,1.492541  ,-0.470819   ,0.436883   ,0.808395   ,0.802536   ,0.345821  ,-0.442825   ,1.500489},
    //   {-0.804934   ,0.516445   ,0.107702  ,-0.135500  ,-0.352635   ,0.523560   ,0.098296  ,-0.065217  ,-0.135727   ,0.176368   ,0.187151  ,-0.380712} };
    // mat theta = mat(4, 12);
    vec targetRLengths = vec(12);
    vec targetDeltaCOM = vec(4);
    double triangleStart[12] = {2.5, 0.5, 0.5, MIN_RL, MIN_RL, MIN_RL, MIN_RL,    MIN_RL, MIN_RL, MIN_RL, MIN_RL, MIN_RL};
    btVector3 initialCOM;


    // /*  NN from 1 face training */
//     double Theta1array[26][4] = {
//       {  3.44074654e-01   ,5.92894115e-01   ,9.71915351e-02   ,7.38811162e-01}
//       ,{  5.74133047e-01  ,-5.59558001e-01  ,-7.17283031e-02  ,-2.06014818e-01}
//       ,{  3.97541273e-01  ,-1.26340252e-01  ,-2.15795608e-01   ,1.50705779e-01}
//       ,{  3.43211175e-01   ,4.64302855e-01  ,-5.36832768e-01  ,-1.09197146e-01}
//       ,{  5.10304974e-01   ,6.50242768e-01   ,5.06926849e-01  ,-4.25744125e-01}
//       ,{  2.01992758e-01   ,3.42831764e-01  ,-2.86908007e-01   ,2.85243521e-01}
//       ,{  5.77606492e-01   ,2.42299423e-01  ,-3.66806915e-01  ,-7.18231356e-01}
//       ,{ -6.60730878e-01  ,-3.86933790e-02   ,1.55093707e-01  ,-7.53730755e-01}
//       ,{ -1.04654033e-03  ,-2.71704650e-01   ,8.27924400e-02   ,1.65506349e-01}
//       ,{ -4.14248472e-01   ,3.43445405e-01   ,4.00300495e-01   ,3.08714923e-01}
//       ,{ -3.11780470e-02   ,5.06361436e-01  ,-1.05539165e+00  ,-1.07411027e+00}
//       ,{  5.80826566e-01   ,2.79660471e-01  ,-3.42335246e-01   ,5.79313746e-02}
//       ,{  7.80288287e-01   ,2.29519813e-03   ,8.04523638e-01   ,1.99842276e-01}
//       ,{  4.15196688e-01   ,1.41945753e-01  ,-1.03227656e+00  ,-1.26047845e-01}
//       ,{  6.45044861e-01   ,2.60156140e-02   ,8.71208905e-02  ,-5.07953283e-01}
//       ,{  4.52905089e-01  ,-1.35400319e-02   ,2.10275017e-01   ,1.00145329e-01}
//       ,{  7.54982339e-01   ,9.05162577e-02  ,-5.13564175e-01   ,1.38447274e-01}
//       ,{ -9.34401820e-03   ,1.32997243e+00   ,1.12608671e+00  ,-3.97592914e-01}
//       ,{  4.28399389e-01   ,2.40049406e-01   ,4.11186425e-02   ,2.57738887e-02}
//       ,{  6.43140768e-01  ,-5.14894183e-01  ,-2.56906386e-02  ,-4.67526100e-01}
//       ,{  1.43227057e-01   ,8.27482661e-01  ,-1.00672766e+00   ,8.55784426e-01}
//       ,{  5.09540980e-01   ,3.34334143e-01   ,4.86837870e-01   ,2.30338982e-01}
//       ,{  5.48261032e-01   ,4.30002649e-01  ,-2.72498074e-01  ,-3.41607895e-01}
//       ,{  3.47719597e-01  ,-6.38330273e-01   ,5.90790519e-01  ,-4.19685307e-01}
//       ,{  7.26928634e-01   ,1.24029501e-01  ,-2.58265606e-01   ,9.26687259e-02}
//       ,{  6.65258871e-01   ,6.37563179e-02   ,2.91654438e-01   ,1.03622675e-01}
//     };
// mat Theta1 = mat(26,4);
//
// double Theta2array[12][26] =
// {
//   {-1.05991047 ,-0.07411006  ,0.67642492  ,0.69345693  ,0.12093734  ,0.2720761
//     ,0.79480453 ,-0.15210751 ,-0.14767118 ,-0.71079274 ,-0.98761223  ,0.15620466
//       ,-0.01657711  ,0.51736037  ,0.87428807  ,0.16536327  ,0.76652649  ,0.36611928
//       ,0.32148201  ,0.02376686 ,-0.8398398  ,-0.08546498  ,0.79803214 ,-0.22133497
//       ,0.6981819   ,0.25347094}
//   ,{ 0.21846956  ,0.38767395  ,0.08369832 ,-0.28909678 ,-0.3703761   ,0.6162795
//     ,-0.65163205 ,-0.24285158  ,0.18644281 ,-1.09526502  ,0.53588288  ,0.00724318
//       ,0.83567724 ,-0.64240904  ,0.27678651  ,0.55996003  ,0.57069817 ,-0.13478247
//       ,0.66789362  ,0.35858877  ,0.15019332  ,0.46970035 ,-0.35810296  ,0.34583739
//       ,0.26482131  ,0.47976439}
//   ,{-0.02231268  ,0.06760041  ,0.34429925  ,0.1453383   ,0.01652688  ,0.40359087
//     ,0.11126091  ,0.03780081  ,0.38590907 ,-0.09102015 ,-0.21645525  ,0.51420495
//       ,0.50518864 ,-0.0522563   ,0.36943465  ,0.2542705   ,0.05735608 ,-0.20885014
//       ,0.41145556  ,0.1173136  ,-0.25862843  ,0.4386724   ,0.20715548  ,0.06291719
//       ,0.28839536  ,0.19926934}
//   ,{-0.27784564 ,-0.09933287  ,0.28012087 ,-0.12320764 ,-0.31782411  ,0.23985271
//     ,0.28665561  ,0.04866318  ,0.19745048 ,-0.10653056 ,-0.38946677  ,0.10050197
//       ,0.51363683  ,0.56894192  ,0.61791745  ,0.17136193  ,0.45754206 ,-0.23212267
//       ,0.38725299  ,0.17572974 ,-0.14294274  ,0.25736649  ,0.29421529  ,0.22524539
//       ,0.03093301  ,0.5934059 ,}
//   ,{ 0.02308605  ,0.22468471  ,0.30580139 ,-0.28751236  ,0.12968407  ,0.12992922
//     ,0.25822938  ,0.11967243  ,0.30026778  ,0.34125618 ,-1.27850963  ,0.39055777
//       ,0.15190281 ,-0.19321607 ,-0.19231387  ,0.44119448  ,0.26632262  ,0.60065007
//       ,0.38787284  ,1.00350872 ,-0.93080685 ,-0.05740478  ,0.39728507  ,0.76160436
//       ,-0.19026281  ,0.1108706 ,}
//   ,{ 0.5917638  ,-0.19502243  ,0.37677667  ,0.99639142 ,-1.26625323  ,0.44200282
//     ,0.15996122  ,0.13553717 ,-0.22753038 ,-0.18508045  ,0.9999875   ,0.59658973
//       ,0.56444567  ,1.06569701  ,0.49383304  ,0.29365612  ,0.71862393 ,-0.43630009
//       ,0.76982359 ,-0.46137748 ,-1.27056955  ,0.45239202  ,0.02467662 ,-0.82626035
//       ,0.38997842  ,0.46603056}
//   ,{-0.21161022 ,-0.51474848  ,0.66311512  ,0.34753594 ,-0.11936578  ,0.06981171
//     ,-0.2180256  ,-0.00202137  ,0.07819019  ,0.2790203  ,-0.09771755  ,0.40569226
//       ,0.25048517  ,0.69729767  ,0.07864395  ,0.45960709  ,0.16695731 ,-0.24314425
//       ,0.62180502 ,-0.1330338  ,-0.18133353  ,0.57447976  ,0.34455925  ,0.34862237
//       ,-0.0707286   ,0.27643321}
//   ,{-0.03703511  ,0.11358634  ,0.62646559 ,-0.39447964 ,-0.17584236  ,0.37439675
//     ,-0.12069764  ,0.06074569  ,0.0637525  ,-0.23782622 ,-0.37844379  ,0.51119773
//       ,0.55201997 ,-0.03788781  ,0.53065833  ,0.20127105  ,0.25182686  ,0.01310919
//       ,0.34163421  ,0.2679761  ,-0.26719327  ,0.13457389  ,0.70314139  ,0.21718236
//       ,0.45480604  ,0.065584  ,}
//   ,{-0.0764719  ,-0.23885056  ,0.66295922  ,0.03894477 ,-0.42325722 ,-0.25707967
//     ,0.0023286  ,-0.06436416  ,0.08071129  ,0.04630522 ,-0.14745554  ,0.11529554
//       ,0.11929462  ,0.20990242  ,0.68959614  ,0.63097889 ,-0.00828752  ,0.07209464
//       ,0.65055722  ,0.45010284 ,-0.21258721  ,0.47258084  ,0.08273051  ,0.05629908
//       ,0.09532718  ,0.30529278}
//   ,{-0.12676972  ,0.02808773  ,0.06960204  ,0.42250499 ,-0.26527767  ,0.07331939
//     ,-0.08109821 ,-0.08397376  ,0.5958243  ,-0.01357493 ,-0.1952417   ,0.3624849
//       ,0.14450241  ,0.10665803  ,0.19277969  ,0.62205143  ,0.2719736  ,-0.31629822
//       ,0.23049027  ,0.120882   ,-0.09186735  ,0.13872199  ,0.46901364  ,0.19776188
//       ,0.42651125  ,0.47639999}
//   ,{-0.45514619  ,0.8782163   ,0.06138197  ,0.34704798 ,-0.70811161  ,0.45022464
//     ,-1.01150365  ,0.33917972  ,0.07742458 ,-0.56485323 ,-0.57484901  ,0.41558134
//       ,-0.03833198  ,0.40333166  ,0.02399095  ,0.35071662  ,0.34359716 ,-1.1175702
//       ,0.77674005  ,0.72572751  ,0.5521322   ,0.44339544  ,0.42700743  ,0.58746576
//       ,0.53148289  ,0.3291471 ,}
//   ,{ 0.15148076 ,-0.90675485  ,0.19235459  ,0.21126873  ,1.20271494 ,-0.20234068
//     ,0.9370769  ,-0.10697567 ,-0.53416859  ,0.46302604 ,-0.09731    ,-0.13822236
//       ,0.48422623 ,-0.36737964  ,0.89156474  ,0.56283755  ,0.07190036 ,-0.63912306
//       ,0.26384274 ,-0.56770662 ,-0.00385657  ,0.61474955  ,0.00628941 ,-0.07705197
//       ,0.37617736  ,0.68226422}
// };
// mat Theta2 = mat(12,26);


    /* NN trained on multiple faces */
mat Theta1 = mat(7,100);
mat Theta2 = mat(100,100);
mat Theta3 = mat(100,100);
mat Theta4 = mat(100,100);
mat Theta5 = mat(100,100);
mat Theta6 = mat(100,100);
mat Theta7 = mat(100,12);
vec Intercept1 = vec(100);
vec Intercept2 = vec(100);
vec Intercept3 = vec(100);
vec Intercept4 = vec(100);
vec Intercept5 = vec(100);
vec Intercept6 = vec(100);
vec Intercept7 = vec(12);

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
