/* USB MIDI Custom Name Example
 *  
 * This example demonstrates how to change the USB MIDI 
 * device name on Teensy LC and 3.x.  When creating more
 * that one MIDI device, custom names are much easier to
 * use when selecting each device in MIDI software on
 * your PC or Mac.  The custom name is in the "name.c" tab.
 *
 * Windows and Macintosh systems often cache USB info.
 * After changing the name, you may need to test on a
 * different computer to observe the new name, or take
 * steps to get your operating system to "forget" the
 * cached info.  (TODO: wanted... can anyone contribute
 * instructions for these systems)
 * 
 * You must select MIDI from the "Tools > USB Type" menu
 * 
  
 
 */
//#include <MIDI.h>
#include <EEPROM.h>
#include <Wire.h>

#define SERIAL_RATE 31250
//#define MIDI1 Serial3

#include <MIDI.h>

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI1);

const int MIDIChannelOut=2; //Message sent on this channel, note 127

unsigned int PreviousState[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int CurrentState[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
bool ChangedFlag=false;
int LoopCounter=0;

int HeaderLength=6;
int HeaderValue[]={0xF0,0x7F,0x00,0x00,0x2D,0x7F,0};
int LoopDelay=3;
int myHeaderFound=false;

int SerialByteCount=0;
char incomingByte=0;
bool bailout=false;

int LEDOut=13;
unsigned long Currenttime=0;
bool WaitingSysex=false;

const int Version=0x0200;


void setup() {
  Serial.begin(115200);

  Serial.println("Setup...");
  MIDI1.begin(MIDI_CHANNEL_OMNI);

  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleSystemExclusive(mySysExChunk);
  
  pinMode(LEDOut,OUTPUT);
  


  
  Serial.print("Version ");Serial.print(Version,HEX);

}




void loop() {

/////////////////////// Check for MIDI in






  
  delay(LoopDelay);
//  digitalWrite(LEDOut,bitRead(LoopCounter,6));



ChangedFlag=false;
//////////////////////////// Send out if changed, Previous=Changed //////////////////////////////////




for (int i=0;i<=15;i++){
    if((CurrentState[i]!=PreviousState[i]))
    {
    ChangedFlag=true;
    }//if changed
}//i  

if(ChangedFlag){
  for (int i=0;i<=15;i++){
    for (int j=0;j<=15;j++){
      if((bitRead(CurrentState[i],j)==1)&&(bitRead(PreviousState[i],j)==0))
      {
          Serial.print("CHANGED Send Note ON:");Serial.print(" "); Serial.print(i,HEX);Serial.print(" "); Serial.println(j,HEX);
		  usbMIDI.sendNoteOn(((i<<4)|j),0x7F,MIDIChannelOut+(i>>15));
      }
      if((bitRead(CurrentState[i],j)==0)&&(bitRead(PreviousState[i],j)==1))
      {
          Serial.print("CHANGED Send Note OFF:");Serial.print(" "); Serial.print(i,HEX);Serial.print(" "); Serial.println(j,HEX);
		  usbMIDI.sendNoteOff(((i<<4)|j),0,MIDIChannelOut+(i>>15));
      }
    }//j
  }//i


  for (int i=0;i<=15;i++){
    PreviousState[i]=CurrentState[i];
  }//i  
  
}//changed flag


/*/ Handling MIDI messages
// 8x, 9x, Ax, Bx, Ex, F2 - pass thru, 3 bytes
//  Cx Dx; F3 F1 - pass thru, 2 bytes
// F6, F8-FF (0b1111 1xxx); - Pass Thru, 1 byte
// Sysex - F0 7F 00 00 2D 7F <Addr> <7 bit Data(bitfield)> <addr> <7 bit data(bitfield)> ... F7

if (MIDI1.available()>0){
	incomingByte=MIDI1.read();
		if((incomingByte&0x80)==0x80
		{
			Serial.print("New Command: ");Serial.println(incomingByte,HEX);
			RxArray[0]=((char) incomingByte);
			bailout=false;
			switch (RxArray[0]){
				case (0x80 to 0xBF):
					while(!MIDI1.available()){}
					RxArray[1]=((char) MIDI1.read());
					while(!MIDI1.available()){}
					RxArray[2]=((char) MIDI1.read());
					break;
				case (0x0C to 0x0D):
					while(!MIDI1.available()){}
					SendSerialByte(MIDI1.read());
					break;
				case (0xE):
					while(!MIDI1.available()){}
					SendSerialByte(MIDI1.read());
					while(!MIDI1.available()){}
					SendSerialByte(MIDI1.read());
					break;
				case (0xF):
					if ((incomingByte&0xD)==1){// F1 or F3
						while(!MIDI1.available()){}
						SendSerialByte(MIDI1.read());
					}
					if ((incomingByte&0xF)==2){// F2 
						while(!MIDI1.available()){}
						SendSerialByte(MIDI1.read());
						while(!MIDI1.available()){}
						SendSerialByte(MIDI1.read());
					}
					if (incomingByte==0xF0)// sysex
					{	incomingByte=0;
						int i=1;
						while(incomingByte!=0xF7){
							while(!MIDI1.available()){}
							incomingByte=MIDI1.read();
							RxArray[i]=((char) incomingByte);
							i++;
						}
						int sysexLength=i-1;
						myHeaderFound=true;
						for (int j=0;j<=(HeaderLength-1);j++0){
							if (RxArray[j]!=HeaderValue[j]) myHeaderFound=false;
						}
						if(myHeaderFound){
							for (int j=HeaderLength; j<sysexLength; j=j+2){
							Serial.print("RxArray[");Serial.print(j,DEC);Serial.print("]: ");Serial.print(RxArray[j],HEX);Serial.print(" - ");Serial.println(RxArray[j+1],HEX);
								if((RxArray[j]&1)==0){
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]&0xFF00;
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]|(((int) RxArray[j+1])&0xFF);
								}
								else{
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]&0xFF;
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]|((((int) RxArray[j+1])&0xFF)<<8);
								}
							}//for j
							
						}//my header is found
						else{
							// normally you'd send the sysex message to usb, but this will never happen in this implementation
						}// my header not found
						
						
						
					}// if sysex
					
					break;


				default: break;


			}//switch
		}//if command
}// if available

*/
//Serial.print(".");
  if (MIDI1.read()) {
    // get a MIDI IN1 (Serial) message
    byte type = MIDI1.getType();
    byte channel = MIDI1.getChannel();
    byte data1 = MIDI1.getData1();
    byte data2 = MIDI1.getData2();
Serial.println("Serial MIDI in");

    // forward the message to USB MIDI virtual cable #0
    if (type != midi::SystemExclusive) {
      // Normal messages, simply give the data to the usbMIDI.send()
      usbMIDI.send(type, data1, data2, channel, 0);
      usbMIDI.send(type, data1, data2, channel, 1);
    } else {
      // SysEx messages are special.  The message length is given in data1 & data2
      unsigned int SysExLength = data1 + data2 * 256;
// const byte??
	  const byte *RxArray=MIDI1.getSysExArray();
//	  int len = MIDI1.getSysArrayLength();
		myHeaderFound=true;
		for (int k=0;k<=(HeaderLength);k++){
			if (RxArray[k]!=HeaderValue[k]) myHeaderFound=false;
			}
			if(myHeaderFound){
				for (int j=HeaderLength+1; j<SysExLength-1; j=j+2){
							Serial.print("RxArray[");Serial.print(j,DEC);Serial.print("]: ");Serial.print(RxArray[j],HEX);Serial.print(" - ");Serial.println(RxArray[j+1],HEX);
								if((RxArray[j]&1)==0){
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]&0xFF00;
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]|(((int) RxArray[j+1])&0xFF);
								}
								else{
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]&0xFF;
									CurrentState[RxArray[j]>>1]=CurrentState[RxArray[j]>>1]|((((int) RxArray[j+1])&0xFF)<<8);
								}
							}//for j
							
			}//my header is found
			else{
				Serial.println("Not My Sysex");
        usbMIDI.sendSysEx(SysExLength, MIDI1.getSysExArray(), true, 0);
			}// my header not found
	  
	  
	  
    }
  }

//Serial.print("-");



  
  LoopCounter++;
 
}//loop  
  

/*void SendSerialByte(int SendByte){
	while(MIDI1.availableForWrite()==0){}
	MIDI1.write((char) SendByte);
}
*/
  
  

void myNoteOn(byte channel, byte note, byte velocity) {
  Serial.print("ExternalNote On, ch=");
  Serial.print(channel, DEC);
  Serial.print(", note=");
  Serial.print(note, DEC);
  Serial.print(", velocity=");
  Serial.println(velocity, DEC);
//  MIDI1.write((unsigned char) (0x90)|((channel)&0xF)); 
//  MIDI1.write((unsigned char) note);
//  MIDI1.write((unsigned char) velocity);

}



void myNoteOff(byte channel, byte note, byte velocity)
{
  Serial.print("External Note Off, ch=");
  Serial.print(channel, DEC);
  Serial.print(", note=");
  Serial.print(note, DEC);
  Serial.print(", velocity=");
  Serial.println(velocity, DEC);
//  MIDI1.write((unsigned char) (0x80)|((channel)&0xF)); 
//  MIDI1.write((unsigned char) note);
//  MIDI1.write((unsigned char) velocity);


}
  
 void MIDI13Byte(int Command,int Data1,int Data2, int Channel){
  Serial.println("Writing to External Serial port");
//  MIDI1.write((unsigned char) (Command&0xF0)|((Channel)&0xF)); 
//  MIDI1.write((unsigned char) Data1);
//  MIDI1.write((unsigned char) Data2);
}


void mySysExChunk(const byte *data, uint16_t SysExlength, bool last){
  Serial.print("SysEx Came In: ");
  WaitingSysex=false;  
  for(int i=0;i<=SysExlength;i++){
    Serial.print(data[i],HEX);
    Serial.print(":");
  }
  Serial.print(" ||| ");
  for(int i=0;i<=SysExlength;i++){
    Serial.write(data[i]);
  }
  Serial.println("");
  
 bool MySysex=false;
 for (int i=1;i<=HeaderLength;i++){
  if(data[i]!=HeaderValue[i]) MySysex=false;  
 }

  if(MySysex){
    for (int i=HeaderLength+1;i<SysExlength;i++){
      if((i&1)==0) CurrentState[i>>1]=data[i]; else CurrentState[(i>>1)+1]=data[i];
    }
  }


}//mySysExChunk
