#include <SDL2/SDL.h>
//#include <SDL2/SDL_controller.h>
#include <iostream>
#include <stdio.h>

enum joyStickAxes {xLeft,yLeft,lTrig,xRight,yRight,rTrigger};
int joyStickVals[6] = {0,0,0,0,0,0};


//Analog joystick dead zone
const int JOYSTICK_DEAD_ZONE = 8000;
//Game Controller 1 handler
SDL_Joystick* gGameController = NULL;


//Function Prototypes go here
void retVals(int jAxisval,int index);
bool init();
void close();
int checkControl();


void retVals(int jAxisval,int index)
{
	if( jAxisval < -JOYSTICK_DEAD_ZONE )
	{
		joyStickVals[index] = -1;
	}
	//Right of dead zone
	else if( jAxisval > JOYSTICK_DEAD_ZONE )
	{
		joyStickVals[index] =  1;
	}
	else
	{
		joyStickVals[index] = 0;
	}
	
}

bool init()
{
    //Initialization flag
    bool success = true;

    //Initialize SDL
    if( SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) < 0 )
    //Can probably skip the initVideo statement
    {
        printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
        success = false;
    }
     //Check for joysticks
	if( SDL_NumJoysticks() < 1 )
	{
		printf( "Warning: No joysticks connected!\n" );
	}
	else
	{
		//Load joystick
		gGameController = SDL_JoystickOpen( 0 );
		printf("Game Controller Found\nYay!!!!!!!!!!!!!!!!\n");
		if( gGameController == NULL )
		{
			printf( "Warning: Unable to open game controller! SDL Error: %s\n", SDL_GetError() );
		}
	}
	
	return(success);

}
	
void close()
{
	//closes open joystick object
	SDL_JoystickClose( gGameController );
    gGameController = NULL; //unlinks memory address of prior gameControlle
    SDL_Quit(); 
    
}
	
int checkControl()
{
	//Event handler
    SDL_Event e;

    //Normalized direction
    bool quit;
    
    while(!quit){
      while( SDL_PollEvent( &e ) != 0 ){
			//User requests quit
			if( e.type == SDL_QUIT )
			{
				quit = true;
			}
			else if( e.type == SDL_JOYAXISMOTION )
			{
				//Motion on controller 0
				//You fuck with this, you fuck it all == keep 0
				if( e.jaxis.which == 0 )
				{                        
					retVals(e.jaxis.value, e.jaxis.axis);
					std::cout<<joyStickVals;
				}
			}
			
		}
	}
	return(0);
}


int main()
{
	init();
	
	std::cout<<checkControl();
		
	close();
	
}
