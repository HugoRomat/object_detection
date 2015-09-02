#include "Object.h"

Object::Object()
{
	//set values for default constructor
	setType("Object");
	setColor(Scalar(0,0,0));

}

Object::Object(string name){

	setType(name);
	
	if(name=="blue"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		//setHSVmin(Scalar(111,106,106));
		//setHSVmax(Scalar(118,176,152));

		//BGR value for Green:
		//setColor(Scalar(135,66,55));

		// Pour Gazebo
		setHSVmin(Scalar(39,40,0));
		setHSVmax(Scalar(212,256,256));
		setColor(Scalar(255,4,4));
	}
	if(name=="green"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(42,0,0));
		setHSVmax(Scalar(104,256,256));

		//BGR value for Yellow:
		setColor(Scalar(32,75,49));

	}
	if(name=="yellow"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(20,176,177));
		setHSVmax(Scalar(42,256,220));

		//BGR value for Red:
		setColor(Scalar(37,150,199));

	}
	if(name=="black"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(20,124,123));
		setHSVmax(Scalar(30,256,256));

		//BGR value for Red:
		setColor(Scalar(0,0,0));

	}
	if(name=="red"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(144,93,129));
		setHSVmax(Scalar(186,256,256));

		//BGR value for Red:
		setColor(Scalar(6,3,106));

	}
}

Object::~Object(void)
{
}

int Object::getXPos(){

	return Object::xPos;

}

void Object::setXPos(int x){

	Object::xPos = x;

}

int Object::getYPos(){

	return Object::yPos;

}

void Object::setYPos(int y){

	Object::yPos = y;

}

Scalar Object::getHSVmin(){

	return Object::HSVmin;

}
Scalar Object::getHSVmax(){

	return Object::HSVmax;
}

void Object::setHSVmin(Scalar min){

	Object::HSVmin = min;
}


void Object::setHSVmax(Scalar max){

	Object::HSVmax = max;
}
