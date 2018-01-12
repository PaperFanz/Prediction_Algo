//------------------------------------------------------------------------------
#include "chai3d.h"
#include <ctime>
#include <ratio>
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a label to display the haptics level (constant of force)
cLabel* labelHapticLevel;

// label to display the scaling multiplier
cLabel* labelHapticScale;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursor;

// a small sphere representing the predicted position
cShapeSphere* predictIndicator[12];

// flag for using force field (ON/OFF)
bool useForceField = true;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

//various settings for the program
double haptics_level = 0.0;
double scale = 1.0;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);

double runningAverage(double prev_a, double a, double operand);
void polyPredict ( vector <double> &axis, double *predictedPos);

//==============================================================================
/*
    Program:   Prediction_Algo

    This application illustrates a threshold based prediction algorithm using
    the Chai3D example program 01-mydevice.cpp as a basis
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "---------------------------------------------" << endl;
    cout << "----------PREDICTION ALGORITHM V1.5----------" << endl;
    cout << "---------------------------------------------" << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable haptic feedback" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[w] - increase haptics level by 1.0" << endl;
    cout << "[s] - decrease haptics level by 1.0" << endl;
    cout << "[e] - increase scale by .1" << endl;
    cout << "[d] - decrease scale by .1" << endl;
    cout << "[esc] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

    #ifdef GLEW_VERSION
    // initialize GLEW
        glewInit();
    #endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (0.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);

    // create an indicator for the predicted position
    for ( int i=0; i<12; i++) 
        predictIndicator[i] = new cShapeSphere(0.003);

    // insert cursor inside world
    world->addChild(cursor);

    // insert prediction indicator into world
    for ( int i=0; i<12; i++){
        world->addChild(predictIndicator[i]);
    }

    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame
        cursor->setShowFrame(true);

        // set the size of the reference frame
        cursor->setFrameSize(0.05);
    }

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a label to display the haptics multiplier
    labelHapticLevel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticLevel);

    // create a label to display the scaling multiplier
    labelHapticScale = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticScale);

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        exit(0);
    }

    // option 1: enable/disable force field
    if (key == '1')
    {
        useForceField = !useForceField;
        if (useForceField)
            cout << "> Enable force field     \r";
        else
            cout << "> Disable force field    \r";
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }

    // options w and s
    if (key == 'w'){
        haptics_level = haptics_level + 1.0;
    }
    if (key == 's'){
        haptics_level = haptics_level - 1.0;
    }


    // options e and d
    if (key == 'e'){
        scale = scale + 0.1;
    }
    if (key == 'd'){
        if( scale - .1 > .001 ){
            scale = scale - 0.1;
        }
        else{
            cout << "Scale cannot equal 0!\n";
        }
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

double runningAverage (double prev_a, double a, double operand)
{
    double avg;
    avg = ((prev_a * (operand - 1) + a)/(operand));
    return avg;
}

//------------------------------------------------------------------------------

void polyPredict ( vector <double> &axis, double *predictedPos)
{

    int i, j, k, n, size;
    n = 3;
    size = axis.size();
    if ( size<10 ) return;
    double y[10];
    for ( int i = 0; i<10; i++ )
        y[i] = axis[size-10+i];
    double X[ 2*n+1 ];                                  //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i=0;i<2*n+1;i++)
    {
        X[i]=0;
        for (j=0;j<10;j++){
            X[i]=X[i]+pow(double(j),i);                      //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
        }
    }

    double B[n+1][n+2],a[n+1];                          //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i=0;i<=n;i++){
        for (j=0;j<=n;j++){
            B[i][j]=X[i+j];                             //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
        }
    }
 
    double Y[n+1];                                      //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<n+1;i++){    
        Y[i]=0;
        for (j=0;j<10;j++)
        Y[i]=Y[i]+pow(double(j),i)*y[j];                //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }

    for (i=0;i<=n;i++)
        B[i][n+1]=Y[i];                                 //load the values of Y as the last column of B(Normal Matrix but augmented)
    n=n+1;                                              //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations  

    for (i=0;i<n;i++)                                   //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=n;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }

    for (i=0;i<n-1;i++)                                 //loop to perform the gauss elimination
        for (k=i+1;k<n;k++)
            {
                double t=B[k][i]/B[i][i];
                for (j=0;j<=n;j++)
                    B[k][j]=B[k][j]-t*B[i][j];          //make the elements below the pivot elements equal to zero or elimnate the variables
            }

    for ( i=0; i<n; i++ )a[i]=0;    
    for (i=n-1;i>=0;i--)                                //back-substitution
    {                                                   //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][n];                                   //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n;j++)
            if (j!=i){
//fprintf(stderr, "%d: %g %g %g\n", i, a[i], B[i][j],a[j]);                                   //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
                a[i]=a[i]-B[i][j]*a[j];
            }
        a[i]=a[i]/B[i][i];           
                   //now finally divide the rhs by the coefficient of the variable to be calculated
    }

    predictedPos[0] = a[0] + (a[1] * 10) + (a[2] * pow( 10, 2 ) + (a[3] * pow( 10, 3)));
    predictedPos[1] = a[0] + (a[1] * 11) + (a[2] * pow( 11, 2 ) + (a[3] * pow( 11, 3)));
    predictedPos[2] = a[0] + (a[1] * 12) + (a[2] * pow( 12, 2 ) + (a[3] * pow( 12, 3)));

}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, windowH - 40, 0);

    // display new position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, windowH - 60, 0);

    // display haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    // display haptic level
    labelHapticLevel->setText( cStr(haptics_level) );

    // update position of label
    labelHapticLevel->setLocalPos(20, windowH - 80, 0);

    // display scale
    labelHapticScale->setText( cStr(scale) ) ;

    //update position of label
    labelHapticScale->setLocalPos(20, windowH - 100, 0);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize frequency counter
    frequencyCounter.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    //initialize logfile
    ofstream logfile;
    logfile.open( "logfile.txt", ios::out | ios::app );

    //vectors for x, y, z, with time components for each
    vector <double> xPos;
    vector <double> yPos;
    vector <double> zPos;
            
    double xPredictedPos[3];
    double yPredictedPos[3];
    double zPredictedPos[3];

    //ideal velocity
    double ideal = 0.4;

    // main haptic simulation loop
    int lastIndicator = 0;
    for ( int count=0; simulationRunning; count++ )
    {
        //reset haptics level when scaling
        if( scale != 1.0 ){haptics_level = 0.0;}

        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////
        // read position
        cVector3d position;
        hapticDevice->getPosition(position);
        position = position * scale;
        double xCoord = position.get(0);
        double yCoord = position.get(1);
        double zCoord = position.get(2);

        //write position to vectors
        if (count%50 == 0){
            double x = position.get(0);
            double y = position.get(1);
            double z = position.get(2);
            xPos.push_back(x);
            yPos.push_back(y);
            zPos.push_back(z);
            predictIndicator[lastIndicator]->setLocalPos(x,y,z);
            if ( ++lastIndicator==9 ) lastIndicator = 0;

            polyPredict( xPos, xPredictedPos );
            polyPredict( yPos, yPredictedPos );
            polyPredict( zPos, zPredictedPos );
            predictIndicator[9]->setLocalPos( xPredictedPos[0], yPredictedPos[0], zPredictedPos[0]);
            predictIndicator[10]->setLocalPos( xPredictedPos[1], yPredictedPos[1], zPredictedPos[1]);
            predictIndicator[11]->setLocalPos( xPredictedPos[2], yPredictedPos[2], zPredictedPos[2]);
        }

        //read rotation
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // tip to gimbal is 3.8cm
        cVector3d tip;
        tip.set( 0.00038,0,0);

        // read linear velocity
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);
        cVector3d normalizedVelocity;
        normalizedVelocity = linearVelocity;
        normalizedVelocity.normalize();
        double cx = linearVelocity.get(0);
        double cy = linearVelocity.get(1);
        double cz = linearVelocity.get(2);
                
        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

        // update position and orientation of cursor
        cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);

        // update global variable for graphic display update
        hapticDevicePosition = position;

        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////
        
        // variables for forces    
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;

        // apply force field
        if ( linearVelocity.length() < ideal )
        {
            cVector3d desiredPosition;
            desiredPosition.set( xPredictedPos[0], yPredictedPos[0], zPredictedPos[0] );
            cVector3d guide = cSub( desiredPosition, position) * haptics_level;
            force.add(guide);
        }
        if ( linearVelocity.length() > ideal )
        {
            // compute linear force
            cVector3d limiter = -cSub(linearVelocity, normalizedVelocity*ideal) * 20;
            force.add(limiter);
        }

        // send computed force, torque, and gripper force to haptic device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // write data to logfiles
        logfile << int(100000*xCoord)<< "\t" << int(100000*yCoord) << "\t" << int(100000*zCoord) << "\t";
        logfile << int(100000*cx) << "\t" << int(100000*cy) << "\t" << int(100000*cz) << "\n";

        // update frequency counter
        frequencyCounter.signal(1);

    }
    // exit haptics thread
    simulationFinished = true;
    logfile.close();
}