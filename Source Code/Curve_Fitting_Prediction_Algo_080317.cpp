//------------------------------------------------------------------------------
#include "chai3d.h"
#include <ctime>
#include <ratio>
#include <chrono>
#include <iostream>
#include <fstream>
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
cShapeSphere* predictIndicator;

// a line representing the velocity vector of the haptic device
cShapeLine* velocity;

// a line representing the average velocity of the haptic device
cShapeLine* avg_velocity;

// flag for using force field (ON/OFF)
bool useForceField = true;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

double haptics_level = 0.0;
double scale = 1.0;
bool position_lock = false;

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

double axisUpperLim(double a, double limit);

double runningAverage(double prev_a, double a, double operand);

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
    cout << "CHAI3D" << endl;
    cout << "Prediction Algorithm v1.0.4" << endl;
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
    predictIndicator = new cShapeSphere(0.005);

    // insert cursor inside world
    world->addChild(cursor);

    // insert prediction indicator into world
    world->addChild(predictIndicator);

    // create small line to illustrate the velocity of the haptic device
    velocity = new cShapeLine(cVector3d(0,0,0), 
                              cVector3d(0,0,0));

    // create a small line to illustrate the average velocity of the haptic device
    avg_velocity = new cShapeLine(cVector3d(0,0,0),
                                  cVector3d(0,0,0));

    // insert lines inside world
    world->addChild(velocity);
    world->addChild(avg_velocity);


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

    // option r: toggle position lock
    if (key == 'r'){
        position_lock = !position_lock;
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

//function for velocity upper limit
double axisUpperLim (double a, double limit)
{
    if( a > limit){
        a = limit;
    }
    if( a < -limit){
        a = -limit;
    }
    return a;
}

//------------------------------------------------------------------------------

double runningAverage (double prev_a, double a, double operand)
{
    double avg;
    avg = ((prev_a * (operand - 1) + a)/(operand));
    return avg;
}

//------------------------------------------------------------------------------

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
    logfile << "Program launched\n";

    //position component values
    double posx;
    double posy;
    double posz;

    //comparative values for current velocity
    double cx;
    double cy;
    double cz;

    //comparative vector for previous velocity
    cVector3d prev_vel;
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    prev_vel.set(px,py,pz);

    //comparative vector for acceleration
    cVector3d accel;
    double delta = .001; //one millisecond base assumption for first prediction
    double delta2 = .000; //unassigned value for logging haptics write speed

    //average values for velocity
    double avgx;
    double avgy;
    double avgz;

    //limit values for x, y, z
    double limx = .05;
    double limy = .05;
    double limz = .05;
    double thresh = .5;

    //count
    int count =  0;
    double operand;

    //declare average linear velocity
    cVector3d avgLinearVelocity;
    
    // main haptic simulation loop
    while(simulationRunning)
    {
        //timer function
        using namespace std::chrono;
        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        //add to count and set operand value
        count = count + 1;
        operand = double(count);

        //reset haptics level when scaling
        if( scale != 1.0 ){
            haptics_level = 0.0;
        }

        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////
        // read position
        cVector3d position;
        hapticDevice->getPosition(position);
        position = position * scale;

        // read orientation
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read gripper position
        double gripperAngle;
        hapticDevice->getGripperAngleRad(gripperAngle);

        // read linear velocity
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);
        cx = axisUpperLim(linearVelocity.get(0), limx) * scale;
        cy = axisUpperLim(linearVelocity.get(1), limy) * scale;
        cz = axisUpperLim(linearVelocity.get(2), limz) * scale;

        //running average calculations
        avgx = runningAverage(avgx, cx, operand);
        avgy = runningAverage(avgy, cy, operand);
        avgz = runningAverage(avgz, cz, operand);

        //discard linear velocity as jitter based on threshold value
        if ( abs(avgx)-abs(cx) >= thresh * scale )
        {
            cx = avgx;
        }

        if ( abs(avgy)-abs(cy) >= thresh * scale )
        {
            cy = avgy;
        }

        if ( abs(avgz)-abs(cz) >= thresh * scale )
        {
            cz = avgz;
        }

        //acceleration calculations
        accel.set((cx-px), (cy-py), (cz-pz));
        accel = accel * delta * scale;

        avgLinearVelocity.set(avgx, avgy, avgz);

        // read angular velocity
        cVector3d angularVelocity;
        hapticDevice->getAngularVelocity(angularVelocity);

        // read gripper angular velocity
        double gripperAngularVelocity;
        hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

        // update arrow
        velocity->m_pointA = position;
        velocity->m_pointB = cAdd(position, cAdd(avgLinearVelocity, accel));
        posx = position.get(0);
        posy = position.get(1);
        posz = position.get(2);

        // update position and orientation of cursor
        cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);

        // update predicted position indicator
        predictIndicator->setLocalPos(cAdd(position, cAdd(avgLinearVelocity, accel)));

        // update global variable for graphic display update
        hapticDevicePosition = position;

        //set prev_vel to curr_vel
        prev_vel.set(cx,cy,cz);

        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d desiredPosition;
        desiredPosition = cAdd(position, cAdd(avgLinearVelocity, accel));

        // desired orientation
        cMatrix3d desiredRotation;
        desiredRotation.identity();
        
        // variables for forces    
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;

        // apply force field
        if (useForceField)
        {
            // compute linear force
            cVector3d forceField = haptics_level * (desiredPosition - position);
            force.add(forceField);
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
        delta = time_span.count();

        // send computed force, torque, and gripper force to haptic device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        //reset count to five so new values are not affecting values too much
        if( count > 200 ){
            count = 5;
        }

        // update frequency counter
        frequencyCounter.signal(1);
        high_resolution_clock::time_point t3 = high_resolution_clock::now();
        duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
        delta2 = time_span2.count();
        logfile << cx << "x " << cy << "y " << cz << "z "  << delta << " seconds before rendering haptics" << delta2 << " seconds for rendering haptics\n";

    }
    // exit haptics thread
    simulationFinished = true;
    logfile.close();
}