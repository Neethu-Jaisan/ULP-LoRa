//AND NAND NOR OR XOR XNOR
//AND
int pinOut = 7; int pinA = 8; int pinB = 9;
void setup()
{ pinMode(pinOut,OUTPUT); 
pinMode(pinA, INPUT); 
pinMode(pinB, INPUT);
} 
void loop()
{ boolean pinAState=digitalRead(pinA); 
boolean pinBState = digitalRead(pinB); 
boolean pinOutState;
// and 
pinOutState =pinAState & pinBState; 
digitalWrite(pinOut,pinOutState);
}

//NAND
int pinOut = 7; int pinA = 8; int pinB = 9;
void setup()
{ pinMode(pinOut,OUTPUT); 
pinMode(pinA, INPUT); 
pinMode(pinB, INPUT);
} 
void loop()
{ boolean pinAState=digitalRead(pinA); 
boolean pinBState = digitalRead(pinB); 
boolean pinOutState;
// and 
pinOutState =!(pinAState & pinBState); 
digitalWrite(pinOut,pinOutState);
}

//OR
int pinOut = 7; int pinA = 8; int pinB = 9;
void setup()
{ pinMode(pinOut,OUTPUT); 
pinMode(pinA, INPUT); 
pinMode(pinB, INPUT);
} 
void loop()
{ boolean pinAState=digitalRead(pinA); 
boolean pinBState = digitalRead(pinB); 
boolean pinOutState;
// and 
pinOutState =pinAState | pinBState; 
digitalWrite(pinOut,pinOutState);
}

//NOR
int pinOut = 7; int pinA = 8; int pinB = 9;
void setup()
{ pinMode(pinOut,OUTPUT); 
pinMode(pinA, INPUT); 
pinMode(pinB, INPUT);
} 
void loop()
{ boolean pinAState=digitalRead(pinA); 
boolean pinBState = digitalRead(pinB); 
boolean pinOutState;
// and 
pinOutState =(pinAState | pinBState); 
digitalWrite(pinOut,pinOutState);
}

//XOR
int pinOut = 7; int pinA = 8; int pinB = 9;
void setup()
{ pinMode(pinOut,OUTPUT); 
pinMode(pinA, INPUT); 
pinMode(pinB, INPUT);
} 
void loop()
{ boolean pinAState=digitalRead(pinA); 
boolean pinBState = digitalRead(pinB); 
boolean pinOutState;
// and 
pinOutState =pinAState ^ pinBState; 
digitalWrite(pinOut,pinOutState);
}

//XNOR
int pinOut = 7; int pinA = 8; int pinB = 9;
void setup()
{ pinMode(pinOut,OUTPUT); 
pinMode(pinA, INPUT); 
pinMode(pinB, INPUT);
} 
void loop()
{ boolean pinAState=digitalRead(pinA); 
boolean pinBState = digitalRead(pinB); 
boolean pinOutState;
// and 
pinOutState =!(pinAState ^ pinBState); 
digitalWrite(pinOut,pinOutState);
}
