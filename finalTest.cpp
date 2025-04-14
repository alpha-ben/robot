#include <iostream>
#include "E101.h"

using namespace std;

// Arrays used for error calculation
int rowT[320]; // array for first row
int mid[320]; // middle array

// Variables to set default speeds for each motor to go forward
unsigned char v_goR = 45;
unsigned char v_goL = 51;

// Arrays for the left and right most column on screen, used for intersection detection
int colL[240];
int colR[240];

//array for bottom row
int rowB[320];

// Variables to store number of Black pixels in left/right most column and top row of image
int L = 0;
int redLine = 0;
int blackPixel;
int topBox = 0;
int leftBox = 0;
int rightBox = 0;
int topRow = 0;
// Counter for intersections
int counter = 1;
//arrays for pillar
int red[320];
int blue[320];
int green[320];

//color vairiables
int b = 0;
int r = 0;
int g = 0;
// Open the gate Function
void openGate() {
    char addr[15] = {'1', '3', '0', '.', '1', '9', '5', '.', '3', '.', '5', '3'};
    int port = 1024;
    char message[24] = "Please";
    char password[24];
    connect_to_server(addr, port);
    send_to_server(message);
    receive_from_server(password);
    send_to_server(password);
}

// Class to define all vehicle related functions
class avcMove {
public:
    // Detect black line function
    void detectLine();

    // Find error value from black pixel array
    int findError();
	int redError();
	int greenError();
	int blueError();
    // Line follow algorithm
    void followLine();
    void followRed();
    void followGreen();
    void followBlue();
    // Check for turning either left or right
	int leftBlack();
	int rightBlack();
	int topBlack();
    // Turn functions
    void turnL();
    void turnR();
    void redPillar();
    void greenPillar();
    void bluePillar();
    
};

void avcMove::detectLine() {
    take_picture();
    update_screen();
    topRow = 0;
    blackPixel = 0;
	
    for (int col = 0; col < 320; col++) {
        if ((int) get_pixel(100, col, 3) < 100) {
            blackPixel++;
            rowT[col] = 1; // black pixel
        } else {
            rowT[col] = 0; // not black pixel
        }
	if ((int) get_pixel(1, col, 3) < 100) {
            topRow++;

    }
}
topBox = topBlack();
 leftBox = leftBlack();
 rightBox = rightBlack();
}


void avcMove::redPillar(){
    take_picture();
    update_screen();
    for (int col = 0; col < 320; col++) {
			 b = (int)get_pixel(100, col, 2);
			 g = (int)get_pixel(100, col, 1);
			 r = (int)get_pixel(100, col, 0);	
			
			if(r > g * 1.5 && r > b * 2){
				
			red[col] = 1; // red pixel
        } else {
            red[col] = 0; // not red pixel
        }

}

}

void avcMove::greenPillar(){
    take_picture();
    update_screen();
    for (int col = 0; col < 320; col++) {
			 b = (int)get_pixel(100, col, 2);
			 g = (int)get_pixel(100, col, 1);
			 r = (int)get_pixel(100, col, 0);	
			
			if(g > r * 1.5 && g > b * 2){
				
			green[col] = 1; // green pixel
        } else {
            green[col] = 0; // not green pixel
        }

}

}


void avcMove::bluePillar(){
    take_picture();
    update_screen();
    for (int col = 0; col < 320; col++){
			 b = (int)get_pixel(100, col, 2);
			 g = (int)get_pixel(100, col, 1);
			 r = (int)get_pixel(100, col, 0);	
			
			if(b > r * 1.5 && b > g * 0.8){
				
			blue[col] = 1; // blue pixel
        } else {
            blue[col] = 0; // not blue pixel
        }

}

}



int avcMove::topBlack(){
	int blackT = 0;
	for (int row = 1; row <20; row++){
		for(int col = 140; col <180; col++){
			if ((int) get_pixel(row, col, 3) < 100) {
            blackT++;
			
			}
		
		} 
	}
return blackT;
}

int avcMove::leftBlack(){
	int blackL = 0;
	for (int row = 80; row <110; row++){
		for(int col = 0; col <20; col++){
			if ((int) get_pixel(row, col, 3) < 110) {
            blackL++;
			
			}
		
		} 
	}
return blackL;
}

int avcMove::rightBlack(){
	int blackR = 0;
	for (int row = 80; row <110; row++){
		for(int col = 300; col <320; col++){
			if ((int) get_pixel(row, col, 3) < 110) {
            blackR++;
			
			}
		
		} 
	}
return blackR;
}

int avcMove::findError() {
    int midPix = 320 / 2;
    int error = 0;
    for (int col = 0; col < 320; col++) {
        mid[col] = col - midPix;
        if (rowT[col] == 1) {
            error += mid[col];
        }
    }
    error = error / (320 * 4);
    return error;
}


int avcMove::redError() {
    int midPix = 320 / 2;
    int error = 0;
    for (int col = 0; col < 320; col++) {
        mid[col] = col - midPix;
        if (red[col] == 1) {
            error += mid[col];
        }
    }
    error = error / (320 * 2);
    return error;
}
int avcMove::greenError() {
    int midPix = 320 / 2;
    int error = 0;
    for (int col = 0; col < 320; col++) {
        mid[col] = col - midPix;
        if (green[col] == 1) {
            error += mid[col];
        }
    }
    error = error / (320 * 2);
    return error;
}

int avcMove::blueError() {
    int midPix = 320 / 2;
    int error = 0;
    for (int col = 0; col < 320; col++) {
        mid[col] = col - midPix;
        if (blue[col] == 1) {
            error += mid[col];
        }
    }
    error = error / (320);
    return error;
}





void avcMove::followLine() {
    int error = findError();
    int dv = 0;
    double kp =0.7, kd = 0.05, dt = 0.5;
    double prevError = 0;
    double derivative = (error - prevError) / dt;
    dv = (kp * error) + (kd * derivative);
    int vL = v_goL + dv;
    int vR = v_goR + dv;  // Correcting direction
	set_motors(5, vL);
    set_motors(3, vR);
    prevError = error;
    hardware_exchange();
    // Check for intersection
  cout<<"error value "<<error<<endl;
  cout<<"dv "<<dv<<endl;
  cout<<"derivative "<<derivative<<endl;
  cout<<"vR "<<vR<<endl;
  cout<<"vL "<<vL<<endl;
            
    }


void avcMove::followRed() {
    int error = redError();
    int dv = 0;
    double kp =0.7, kd = 0.05, dt = 0.5;
    double prevError = 0;
    double derivative = (error - prevError) / dt;
    dv = (kp * error) + (kd * derivative);
    int vL = v_goL + dv;
    int vR = v_goR + dv;  // Correcting direction
	set_motors(5, vL);
    set_motors(3, vR);
    prevError = error;
    hardware_exchange();
    

  cout<<"error value "<<error<<endl;
  cout<<"dv "<<dv<<endl;
  cout<<"derivative "<<derivative<<endl;
  cout<<"vR "<<vR<<endl;
  cout<<"vL "<<vL<<endl;
    }

void avcMove::followGreen() {
    int error = greenError();
    int dv = 0;
    double kp =0.7, kd = 0.05, dt = 0.5;
    double prevError = 0;
    double derivative = (error - prevError) / dt;
    dv = (kp * error) + (kd * derivative);
    int vL = v_goL + dv;
    int vR = v_goR + dv;  // Correcting direction
	set_motors(5, vL);
    set_motors(3, vR);
    prevError = error;
    hardware_exchange();
    // Check for intersection
  cout<<"error value"<<error<<endl;
            
    }

void avcMove::followBlue() {
    int error = blueError();
    int dv = 0;
    double kp =0.7, kd = 0.05, dt = 0.5;
    double prevError = 0;
    double derivative = (error - prevError) / dt;
    dv = (kp * error) + (kd * derivative);
    int vL = v_goL + dv;
    int vR = v_goR + dv;  // Correcting direction
	set_motors(5, vL);
    set_motors(3, vR);
    prevError = error;
    hardware_exchange();
    // Check for intersection
  cout<<"error value"<<error<<endl;
            
    }


void avcMove::turnL() {
        // right motor runs while left motor is off
        set_motors(3, 38);
        set_motors(5, 48);
        hardware_exchange();

}

void avcMove::turnR() {
        // left motor runs while right motor is off
        set_motors(3, 48);
        set_motors(5, 58);
        hardware_exchange();

}

void cameraUp() {
    // Convert the desired angle to the corresponding PWM value
    //unsigned char pwm_value = static_cast<unsigned char>(angle);  // Adjust conversion as needed
    set_motors(1,30);  // Assuming motor channel 1 is used for camera angle
    hardware_exchange();
    //cout << "Camera angle shifted " << angle << " degrees up" << endl;
}

void cameraDown(){
set_motors(1,60);
hardware_exchange();

}
void stop(){
set_motors(3, 48);
        set_motors(5, 48);
        hardware_exchange();
sleep1(2000);
}
void forward(){
set_motors(3, 31);
        set_motors(5, 61);
        hardware_exchange();
sleep1(2000);

}

// main function
int main() {
    init(0);
    avcMove avc;
    cameraDown();
    while (1) {
        int quad = 1; // quadrant number vehicle is currently in

        switch (quad) {
            case 1: {
                // Need to open gate and proceed to quadrant 2
cameraDown();               
 openGate();
for(int i = 0; i<10; i++){
v_goR = 31;
v_goL = 61;
avc.detectLine();
avc.followLine();
sleep1(200);
}
               
                // Change to quadrant 2
               quad++; 
                
            }

            case 2: {
v_goR = 45;
v_goL = 51;


				counter = 0;
                // Follow the black line around the course to get to quadrant 3
                while (quad == 2) {
					avc.detectLine();
					avc.followLine();
					int redPixel = 0;
					int bluePixel = 0;
					int greenPixel = 0;
					
				// for all pixels in latest image
					redLine = 0;
				// for all pixels in latest image
				   
							for (int col = 0; col < 320; col++) {
									 bluePixel = (int)get_pixel(10, col, 2);
									 greenPixel = (int)get_pixel(10, col, 1);
									 redPixel = (int)get_pixel(10, col, 0);        
									
									if(redPixel > greenPixel * 1.5 && redPixel > bluePixel * 2){
											redLine ++ ;
									}
							}
        
    cout<<"No. of red pixels: "<<redLine<<endl;
    
    if (redLine >10 ) {  // Threshold for number of red pixels
            // Shift camera angle 50 degrees up
		cout<<"CHANGE QUADRANT"<<endl;
			quad++;
        }
	}
}
	
case 3:{
cout<<"QUADRANT 3 CODE"<<endl;
counter = 0;
//detect and follow the pillars to reach the red ball
		while (quad == 3){
					avc.detectLine();
					avc.followLine();

		
cout<<"top row "<< topRow<<endl;
cout<<"left "<< leftBox<<endl;
cout<<"right "<< rightBox<<endl;
cout<<"blackPixel in follow line "<< blackPixel<<endl;
						if (counter == 0){
							//90 degree right
							if (rightBox>300 && leftBox<5 ){
							avc.turnR();
							cout<<"first turn right"<<endl;
							sleep1(1000);
							counter++;
						}
						}
						else if(counter ==1){
							//90 degree right
							if (rightBox>200&& leftBox<5){
							avc.turnR();
							cout<<"second, turn right"<<endl;
							sleep1(1000);
							counter++;
						}
						}
						else if(counter ==2){
							//90 degreee left
							if (rightBox<5 && leftBox> 200){
							avc.turnL();
							cout<<"third turn left"<<endl;
							
							sleep1(1000);
							counter++;
						}
						}
						else if(counter ==3){
						 //first intersection
							if (rightBox>200&& leftBox>200){
							avc.turnL();
							cout<<"fourth turn left"<<endl;
							sleep1(1000);
							counter++;
						}
						}
						else if(counter ==4){
						//90 degree right
							if (rightBox>200&& leftBox<5 ){
							avc.turnR();
							cout<<" turn 5 right"<<endl;
							sleep1(1000);
							counter++;
						}
						}
						else if(counter ==5){
						 //second intersection
							if (rightBox>200&& leftBox>200 ){
							avc.turnL();
							cout<<"turn 6 left"<<endl;
							sleep1(1500);
							counter++;
						}
						}
						else if(counter ==6){
						//last turn, the final intersection
						if (rightBox>200&& leftBox>200){
							avc.turnR();
							cout<<"turn 7 right"<<endl;
							sleep1(1000);
							counter++;
						}
						}else if (counter ==7){
							  take_picture();
        update_screen();
        int redPixel = 0;
        int bluePixel = 0;
        int greenPixel = 0;
        int redPixelCount = 0;
    // for all pixels in latest image
    for (int row = 0 ; row < 240 ; row++) {     
                for (int col = 0; col < 320; col++) {
                         bluePixel = (int)get_pixel(row, col, 2);
                         greenPixel = (int)get_pixel(row, col, 1);
                         redPixel = (int)get_pixel(row, col, 0);        
                        
                        if(redPixel > greenPixel * 1.5 && redPixel > bluePixel * 2){
                                redPixelCount ++ ;
                        }
                }
        }
    cout<<"No. of red pixels: "<<redPixelCount<<endl;
    if (redPixelCount > 1000) {  // Threshold for number of red pixels
            // Shift camera angle 50 degrees up
 cameraUp();
                    set_motors(3, 31);
                  set_motors(5, 61);
sleep1(2000);
                    
                   quad++;
							}
							
                // Change to quadrant 4
      


        }
}

}

//change angle of the camera
case 4:{
//red pillar
	int pillar = 1;
	switch(pillar){
  case 1: {
  while(pillar == 1){
  // make 1000 runs  
  	
	int redPixel = 0;
	int bluePixel = 0;
	int greenPixel = 0;
	int redPixelCount = 0;
    // for all pixels in latest image
    
    while(redPixelCount < 5000){
		take_picture();
	update_screen();
	// keep turning right
	             avc.turnR();
    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			if(redPixel > greenPixel * 1.5 && redPixel > bluePixel * 2){
				redPixelCount ++ ;
			}
		}
	}
}
	stop();
forward();
    cout<<"No. of red pixels: "<<redPixelCount<<endl;
    while(redPixelCount < 30000){
		 redPixelCount=0;
		 take_picture();
	update_screen();
		 //set_motors(3, 40);
	     //set_motors(5, 56);
	     //hardware_exchange();
	     avc.redPillar();

	     avc.followRed();
		cout<<"The robot move"<< endl;
		    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			if(redPixel > greenPixel * 1.5 && redPixel > bluePixel * 2){
				redPixelCount ++ ;
			}
			
		}
	}
		
cout<<"No. of red pixels: "<<redPixelCount<<endl;
	}
	
		 set_motors(3, 48);
	                set_motors(5, 48);
	                  hardware_exchange();
		cout<<"The robot stop"<<endl;

	cout<<"pillar1"<<endl;
	cout<<"============"<<endl;
avc.turnL();
sleep1(500);

break;
}
	pillar++;
}
case 2:{
	
	cout<<"pillar2"<<endl;
	 avc.turnL();
sleep1(1000);   
	while(pillar == 2){
  // make 1000 runs  
  	
	int redPixel = 0;
	int bluePixel = 0;
	int greenPixel = 0;
	int greenPixelCount = 0;
    // for all pixels in latest image
    
    while(greenPixelCount <1000){
		take_picture();
	update_screen();
	// keep turning left
	               avc.turnL();
    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			if(greenPixel > redPixel * 1.5 && greenPixel > bluePixel * 2){
				greenPixelCount ++ ;
			}
		}
	}
}
	
    cout<<"No. of green pixels: "<<greenPixelCount<<endl;
stop();
forward();
    while(greenPixelCount < 30000){
		 greenPixelCount=0;
		 take_picture();
	update_screen();
		 //set_motors(3, 40);
	     //set_motors(5, 56);
	     //hardware_exchange();
	     avc.greenPillar();
	     avc.followGreen();
		cout<<"The robot move"<< endl;
		    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			
			if(greenPixel > redPixel * 1.5 && greenPixel > bluePixel * 2){
				greenPixelCount ++ ;
			}
			
		}
	}
		
cout<<"No. of green pixels: "<<greenPixelCount<<endl;
	}
	
		 set_motors(3, 48);
	                set_motors(5, 48);
	                  hardware_exchange();
		cout<<"The robot stop"<<endl;

	cout<<"pillar2"<<endl;
	cout<<"============"<<endl;
avc.turnR();
sleep1(500);
break;
}
	pillar++;
}
case 3:{
	
	cout<<"pillar3"<<endl;
	 avc.turnR();
sleep1(1000);   
	while(pillar == 3){
  // make 1000 runs  
  	
	int redPixel = 0;
	int bluePixel = 0;
	int greenPixel = 0;
	int bluePixelCount = 0;
    // for all pixels in latest image
    
    while(bluePixelCount <1000){
		take_picture();
	update_screen();
	// keep turning right
	              avc.turnR();
    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			if(bluePixel > redPixel * 1.5 && bluePixel > greenPixel * 0.8){
				bluePixelCount ++ ;
			}
		}
	}
}
	
    cout<<"No. of blue pixels: "<<bluePixelCount<<endl;
stop();
forward();
    while(bluePixelCount < 20000){
		 bluePixelCount=0;
		 take_picture();
	update_screen();
		 //set_motors(3, 40);
	     //set_motors(5, 56);
	     //hardware_exchange();
	     avc.bluePillar();
	     avc.followBlue();
		cout<<"The robot move"<< endl;
		    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			
				if(bluePixel > redPixel * 1.5 && bluePixel > greenPixel * 0.8){
				bluePixelCount ++ ;
			}
			
		}
	}
		
cout<<"No. of blue pixels: "<<bluePixelCount<<endl;
	}
	
		 set_motors(3, 48);
	                set_motors(5, 48);
	                  hardware_exchange();
		cout<<"The robot stop"<<endl;

	cout<<"pillar3"<<endl;
	cout<<"============"<<endl;
avc.turnL();
sleep1(500);
break;
}
	pillar++;
	}
	//knock red ball off the table and stop the program 
case 4:{
	 avc.turnL();
sleep1(1000);   
	while(pillar == 4){

  	
	int redPixel = 0;
	int bluePixel = 0;
	int greenPixel = 0;
	int redPixelCount = 0;
    // for all pixels in latest image
    
    while(redPixelCount <10000){
		take_picture();
	update_screen();
	// keep turning left
	                avc.turnL();
    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			if(redPixel > greenPixel * 1.5 && redPixel > bluePixel * 2){
				redPixelCount ++ ;
			}
		}
	}
}
	
    cout<<"No. of red pixels: "<<redPixelCount<<endl;
stop();
forward();
    //when the vehicl is moving towards and then pushing until it sees less than 1000 red pixels
    while(redPixelCount > 1000){
		 redPixelCount=0;
		 take_picture();
	update_screen();
		 //set_motors(3, 40);
	     //set_motors(5, 56);
	     //hardware_exchange();
	     avc.redPillar();
	     avc.followRed();
		cout<<"The robot move"<< endl;
		    for (int row = 0 ; row < 240 ; row++) {	
		for (int col = 0; col < 320; col++) {
			 bluePixel = (int)get_pixel(row, col, 2);
			 greenPixel = (int)get_pixel(row, col, 1);
			 redPixel = (int)get_pixel(row, col, 0);	
			
			if(redPixel > greenPixel * 1.5 && redPixel > bluePixel * 2){
				redPixelCount ++ ;
			}
			
		}
	}
		
cout<<"No. of red pixels: "<<redPixelCount<<endl;
	}//automatically exit loop when <1000 red pixels detected
	//stop the robot
		 set_motors(3, 48);
	     set_motors(5, 48);
	                  hardware_exchange();
		cout<<"The robot stop"<<endl;
//exit the whole code
	cout<<"pillar4"<<endl;
	exit(1);//stops the whole code
break;//exit while pillar == 4 
}
	
	
	break;//exit pillar = 4 case
	}
    



//Must push red ball of table and stop code

break;//exit Pillar switch
}
break;//exit quadrant 4

}
break;//exit out of main quadrant changeing switch
}
break;//exit out of while(1)
}

 close_screen_stream();

return 0;//end the code

}


