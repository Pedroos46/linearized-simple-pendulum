// SET INITIAL VARIABLES. Question 2.1.1
// INSERT YOUR CODE HERE.
float StateTheta = radians(30.0); 
float StateDTheta = 0.0;
float StateATheta= 0.0;

//////////////////////////////////////////////////////////////////////////////

// STATE DEFINITION. Question 2.1.2
// INSERT YOUR CODE HERE
int Arraysize = 3;
int StTh = 0;
int StDTh = 1;
int StATh = 2;

float [] statePedulN = new float [Arraysize];

float [] stateEuler = new float [Arraysize];
float [] stateEulerC = new float [Arraysize];
float [] stateRK2 = new float [Arraysize];
float [] stateRK4 = new float [Arraysize];

//////////////////////////////////////////////////////////////////////////////

//Window set up variables
int WindowWidthHeight = 300;
float WorldSize = 2.0;
float PixelsPerMeter;
float OriginPixelsX;
float OriginPixelsY;

//Global constants
float g = -9.8;// gravity = 9.8 m/s^2
float PendulumLength = 1.0;// Length of the pendulum arm, in meters. This is constant, and therefore not really part of the simulation state


void setup(){
    // CREATE INITIAL STATES. Question 2.1.3
    // INSERT YOUR CODE HERE
    stateEuler[StTh]= StateTheta ;
    stateEuler[StDTh]= StateDTheta;
    stateEuler[StATh]= StateATheta;
    
    statePedulN[StTh] = StateTheta;
    statePedulN[StDTh] = StateDTheta;
    statePedulN[StATh] = StateATheta;
    
    stateEulerC[StTh] = StateTheta;
    stateEulerC[StDTh] = StateDTheta;
    stateEulerC[StATh] = StateATheta;
    
    stateRK2[StTh] = StateTheta;
    stateRK2[StDTh] = StateDTheta;
    stateRK2[StATh] = StateATheta;
    
    stateRK4[StTh] = StateTheta;
    stateRK4[StDTh] = StateDTheta;
    stateRK4[StATh] = StateATheta;
/////////////////////////////////////////////////////////////////////////////    
   
    // Set up normalized colors.
    colorMode( RGB, 1.0 );
    
    // Set up the stroke color and width.
    stroke( 0.0 );
    //strokeWeight( 0.01 );
    
    // Create the window size, set up the transformation variables.
    size( 720, 720 );
    PixelsPerMeter = (( float )WindowWidthHeight ) / WorldSize;
    OriginPixelsX = 0.25 * ( float )WindowWidthHeight;
    OriginPixelsY = 0.25 * ( float )WindowWidthHeight;
    
}

void timeStep(float delta_t){
   //PENDUL NORMAL
   statePedulN[StDTh]= statePedulN[StDTh] + delta_t;
   statePedulN[StTh]= statePedulN[StTh] + delta_t * statePedulN[StATh];
   statePedulN[StATh]= statePedulN[StATh] + delta_t * g * sin (statePedulN[StTh]);
  
//////////////////////////////////////////////////////////////////////////////
  
   // EULER METHOD. Question 2.2.2
   // INSERT YOUR CODE HERE 
   StateATheta = (g)/(PendulumLength)*sin(stateEuler[StTh]);
   StateDTheta = stateEuler[StDTh] + (delta_t *StateATheta);
   StateTheta = stateEuler[StTh] + (delta_t * stateEuler[StDTh]);
  
   stateEuler[StTh] = StateTheta;
   stateEuler[StDTh]= StateDTheta;
  
  
/////////////////////////////////////////////////////////////////////////////
  
   // EULER-CROMER METHOD. Question 2.2.2
   // INSERT YOUR CODE HERE
   
   float acceleracio = ( g / PendulumLength ) * stateEulerC[StTh]; // Calculem acceleració a partir de la posició actual. 
   stateEulerC[StDTh] += delta_t * acceleracio; // Actualitzem velocitat a partir de la acceleració.
   stateEulerC[StTh] += delta_t * stateEulerC[StDTh]; // Actualitzem posició a partir de la velocitat actualitzada.
   stateEulerC[StATh] += delta_t; // Actualitzem el temps.
  
/////////////////////////////////////////////////////////////////////////////
 
   // RK_2. Question 2.2.2
   // INSERT YOUR CODE HERE
   float vPas1 = stateRK2[StDTh];
   float aPas1 = (g/PendulumLength ) * sin(stateRK2[StTh]);
   float vPas2 = stateRK2[StDTh] + ( delta_t * aPas1 );
   
   float xTmp = stateRK2[StTh] + ( delta_t * vPas1 );
   
   float aPas2 = (g/PendulumLength ) * sin(xTmp);
   
   stateRK2[StTh] += ( delta_t / 2.0 ) * ( vPas1 + vPas2 );
   stateRK2[StDTh] += ( delta_t / 2.0 ) * ( aPas1 + aPas2 );
   
   stateRK2[StATh] += delta_t;// Actualitzar el temps.
  
/////////////////////////////////////////////////////////////////////////////
  
   // RK_4. Question 2.2.2
   // INSERT YOUR CODE HERE
  
   float vPas11 = stateRK4[StDTh];
   float aPas11 = (g/PendulumLength ) * sin( stateRK4[StTh] );
  
   float vPas21 = stateRK4[StDTh] + ( ( delta_t / 2.0 ) * aPas11 ); 
   float xTmp2 = stateRK4[StTh] + ( ( delta_t / 2.0 ) * vPas11 );
   float aPas21 = (g/PendulumLength ) * sin( xTmp2 );
   
   float vPas3 = stateRK4[StDTh] + ( ( delta_t / 2.0 ) * aPas21 );
   float xTmp3 = stateRK4[StTh] + ( ( delta_t / 2.0 ) * vPas21 );
   float aPas3 = (g/PendulumLength ) * sin( xTmp3 );
   
   float vPas4 = stateRK4[StDTh] + ( delta_t * aPas3 );
   float xTmp4 = stateRK4[StTh] + ( delta_t * vPas3 );
   float aPas4 = (g/PendulumLength ) * sin( xTmp4 );
   
   stateRK4[StTh] += ( delta_t / 6.0 ) *
   (vPas1 + (2.0*vPas2) + (2.0*vPas3) + vPas4 );
   
   stateRK4[StDTh] += ( delta_t / 6.0 ) *
   (aPas1 + (2.0*aPas2) + (2.0*aPas3) + aPas4 );
   
   stateRK4[StATh] += delta_t;   // Actualitzar el temps.
  
/////////////////////////////////////////////////////////////////////////////

}

// The DrawState function assumes that the coordinate space is that of the
// simulation - namely, meters, with the pendulum pivot placed at the origin.
// Draw the arm, pivot, and bob!
// There is currently a bug in processing.js which requires us to do the
// pixels-per-meter scaling ourselves.
void DrawState(){ 
   float offsetX = 100;
   float offsetY = 100;
   float offsetECX = 460;
   float offsetECY = 100;
   float offsetRK2X = 100;
   float offsetRK2Y = 460;
   float offsetRK4X = 460;
   float offsetRK4Y = 460;

/////////////////////////////////////////////////////////////////////////////

   // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE EULER SOLUTION. Question 2.2.3
   // INSERT YOUR CODE HERE
    
   float ArmEndX = PixelsPerMeter * PendulumLength * sin(StateTheta);//Replace the zero inside the parenthesis with your code
   float ArmEndY = PixelsPerMeter * PendulumLength * cos(StateTheta);//Replace the zero inside the parenthesis with your code
   strokeWeight( 1.0 );
   line( offsetX, offsetY, ArmEndX+offsetX, ArmEndY+offsetY );
          
   // Draw the pendulum pivot
   fill( 0.0 );
   ellipse( offsetX, offsetY, 
            PixelsPerMeter * 0.03, 
            PixelsPerMeter * 0.03 );
 
   // Draw the pendulum bob
   fill( 1.0, 0.0, 0.0 );
   ellipse( ArmEndX+offsetX, ArmEndY+offsetY, 
            PixelsPerMeter * 0.1, 
            PixelsPerMeter * 0.1 );
    
   drawSimplePendul(offsetX, offsetY);

/////////////////////////////////////////////////////////////////////////////
    
   // DRAW THE EULER-CROMER SOLUTION. Question 2.2.3
   // INSERT YOUR CODE HERE
       
   float ArmEndECX = PixelsPerMeter * PendulumLength * sin(stateEulerC[StTh]);//Replace the zero inside the parenthesis with your code
   float ArmEndECY = PixelsPerMeter * PendulumLength * cos(stateEulerC[StTh]);//Replace the zero inside the parenthesis with your code
     
   strokeWeight( 1.0 );
   line( offsetECX, offsetECY, ArmEndECX+offsetECX, ArmEndECY+offsetECY );
   fill( 0.0 );
   ellipse( offsetECX, offsetECY, 
            PixelsPerMeter * 0.03, 
            PixelsPerMeter * 0.03 );
 
   // Draw the pendulum bob
   fill( 1.0, 0.0, 0.0 );
   ellipse( ArmEndECX+offsetECX, ArmEndECY+offsetECY, 
            PixelsPerMeter * 0.1, 
            PixelsPerMeter * 0.1 );   
      
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           

   // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE EULER-CROMER SOLUTION. Question 2.2.3
   // INSERT YOUR CODE HERE   
    
   drawSimplePendul(offsetECX, offsetECY);

             
/////////////////////////////////////////////////////////////////////////////
    
   // DRAW THE RK_2. Question 2.2.3
   // INSERT YOUR CODE HERE   
   float ArmEndRK2X = PixelsPerMeter * PendulumLength * sin(stateRK2[StTh]);//Replace the zero inside the parenthesis with your code
   float ArmEndRK2Y = PixelsPerMeter * PendulumLength * cos(stateRK2[StTh]);//Replace the zero inside the parenthesis with your code
     
   strokeWeight( 1.0 );
   line( offsetRK2X, offsetRK2Y, ArmEndRK2X+offsetRK2X, ArmEndRK2Y+offsetRK2Y );
   fill( 0.0 );
   ellipse( offsetRK2X, offsetRK2Y, 
            PixelsPerMeter * 0.03, 
            PixelsPerMeter * 0.03 );
 
   // Draw the pendulum bob
   fill( 1.0, 0.0, 0.0 );
   ellipse( ArmEndRK2X+offsetRK2X, ArmEndRK2Y+offsetRK2Y, 
            PixelsPerMeter * 0.1, 
            PixelsPerMeter * 0.1 );
      
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     

   // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE RK-2 SOLUTION. Question 2.2.3
   // INSERT YOUR CODE HERE    
    
   drawSimplePendul(offsetRK2X, offsetRK2Y);    
   
/////////////////////////////////////////////////////////////////////////////
    
   // DRAW THE RK_4. Question 2.2.3
   // INSERT YOUR CODE HERE   
   float ArmEndRK4X = PixelsPerMeter * PendulumLength * sin(stateRK4[StTh]);//Replace the zero inside the parenthesis with your code
   float ArmEndRK4Y = PixelsPerMeter * PendulumLength * cos(stateRK4[StTh]);//Replace the zero inside the parenthesis with your code
    
   strokeWeight( 1.0 );
   line( offsetRK4X, offsetRK4Y, ArmEndRK4X+offsetRK4X, ArmEndRK4Y+offsetRK4Y );
   fill( 0.0 );
   ellipse( offsetRK4X, offsetRK4Y, 
            PixelsPerMeter * 0.03, 
            PixelsPerMeter * 0.03 );
 
   // Draw the pendulum bob
   fill( 1.0, 0.0, 0.0 );
   ellipse( ArmEndRK4X+offsetRK4X, ArmEndRK4Y+offsetRK4Y, 
            PixelsPerMeter * 0.1, 
            PixelsPerMeter * 0.1 );

      
   //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      

   // DRAW THE EXACT SOLUTION IN CONJUNCTION WITH THE RK-4 SOLUTION. Question 2.2.3
   // INSERT YOUR CODE HERE    
    
   drawSimplePendul(offsetRK4X, offsetRK4X); // Funciona
    
/////////////////////////////////////////////////////////////////////////////    
             
}
    
  
void drawSimplePendul(float X, float Y){
   //PENDULO SIMPLE
   float ArmEndX2 = PixelsPerMeter * PendulumLength * sin(statePedulN[StTh]);//Replace the zero inside the parenthesis with your code
   float ArmEndY2 = PixelsPerMeter * PendulumLength * cos(statePedulN[StTh]);//Replace the zero inside the parenthesis with your code

   // Draw the pendulum arm.
   strokeWeight( 1.0 );
   line( X, Y+20, ArmEndX2+X, ArmEndY2+Y+20 );
          
   // Draw the pendulum pivot
   fill( 0.0 );
   ellipse( X, Y+20, 
            PixelsPerMeter * 0.03, 
            PixelsPerMeter * 0.03 );
    
   // Draw the pendulum bob
   fill(#2196f3);
   ellipse( ArmEndX2+X, ArmEndY2+Y+20, 
            PixelsPerMeter * 0.1, 
            PixelsPerMeter * 0.1 );
            
}

// The draw function creates a transformation matrix between pixel space
// and simulation space, in meters, and then calls the DrawState function.
// Unfortunately, there is currently a bug in processing.js with concatenated
// transformation matrices, so we have to do the coordinate scaling ourselves
// in the draw function.
void draw(){
   //Time Step
   timeStep(1.0/24.0);
  
   //Clear the display to a constant color
   background( 0.75 );
    
   // Translate to the origin.
   translate( OriginPixelsX, OriginPixelsY );
    
   // DRAW THE QUADRANTS AND NAMES. Question 2.3.1
   // INSERT YOUR CODE HERE
   // Add additional code here (if not done in the DrawState() function) to draw the quadrants and names.        
   
   line (-100,280,1000,280);//linea horizontal
   line (280,-100,280,1000);// linea vertical
   //titols
   textSize(32);
   fill(0, 0, 153, 51);
   text("Euler", -70, -50); 
   text("Euler-Cromer", 300, -50);
   text("RK 2", -70, 320);
   text("RK 4", 300, 320); 

   // Draw the simulation
   DrawState();
}
