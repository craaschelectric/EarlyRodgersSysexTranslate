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
#define MIDI1 Serial3

#include <MIDI.h>

//MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI1);

const int MIDIChannelOut=2; //Message sent on this channel, note 127

unsigned int PreviousState[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int CurrentState[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
bool ChangedFlag=false;
int LoopCounter=0;

char RxArray[]={0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 
0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0};//12*8 bytes
int sysexLength=0;

int HeaderLength=7;
int HeaderValue[]={0xF0,0x7F,0x00,0x00,0x2D,0x7F,0};
int LoopDelay=3;
int myHeaderFound=false;

int SerialByteCount=0;
char incomingByte=0;
bool bailout=false;

int LEDOut=13;
unsigned long Currenttime=0;
bool WaitingSysex=false;
int Channel=0;

const int Version=0x0200;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, MIDI);

void setup() {
  Serial.begin(115200);

  MIDI.begin(MIDI_CHANNEL_OMNI);
//  MIDI.begin(31250);

  Serial.println("Setup...");
//  MIDI.begin(MIDI_CHANNEL_OMNI);

  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleSystemExclusive(mySysExChunk);
  
  pinMode(LEDOut,OUTPUT);
  


  
  Serial.print("Version ");Serial.print(Version,HEX);

}




void loop() {

/////////////////////// Check for MIDI in






  
//  delay(LoopDelay);
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
  Serial.print("Previous State: ");
  for (int i=15;i>=0;i--){Serial.print(PreviousState[i],HEX);Serial.print(":");}
  Serial.println("");
  Serial.print("Current State: ");
  for (int i=15;i>=0;i--){Serial.print(CurrentState[i],HEX);Serial.print(":");}
  
  for (int i=0;i<=15;i++){
    for (int j=0;j<=15;j++){
      if((bitRead(CurrentState[i],j)==1)&&(bitRead(PreviousState[i],j)==0))
      {
          Serial.print("CHANGED Send Note ON:");Serial.print(" "); Serial.print(i,HEX);Serial.print(" "); Serial.print(j,HEX);Serial.print(" : Note ");
      Serial.print(((i<<4)|j)&0x7F,HEX);Serial.print(" Channel ");Serial.println(MIDIChannelOut+(i>>3),HEX);
      usbMIDI.sendNoteOn(((i<<4)|j)&0x7F,0x7F,MIDIChannelOut+(i>>3));
      }
      if((bitRead(CurrentState[i],j)==0)&&(bitRead(PreviousState[i],j)==1))
      {
          Serial.print("CHANGED Send Note OFF:");Serial.print(" "); Serial.print(i,HEX);Serial.print(" "); Serial.print(j,HEX);Serial.print(" : Note ");
      Serial.print(((i<<4)|j)&0x7F,HEX);Serial.print(" Channel ");Serial.println(MIDIChannelOut+(i>>3),HEX);
      usbMIDI.sendNoteOff(((i<<4)|j)&0x7F,0,MIDIChannelOut+(i>>3));
      }
    }//j
  }//i


  for (int i=0;i<=15;i++){
    PreviousState[i]=CurrentState[i];
  }//i  
  
}//changed flag

/*if(MIDI.available()>0){
  incomingByte=MIDI.read();
  Serial.println(incomingByte,HEX);
}
*/

//Serial.print(".");
	
  if(MIDI.read()) 
    {
      RxArray[0]=((char) MIDI.getType());
      if(RxArray[0]!=0xFE){
        Serial.print("New Command: ");Serial.print(RxArray[0],HEX);Serial.println(" ");
      }
      bailout=false;

      if ((RxArray[0]>=0x80)&&(RxArray[0]<=0xBF)){
           Channel=MIDI.getChannel();
          RxArray[1]=((char) MIDI.getData1());
          Serial.print(RxArray[1],HEX);Serial.print(" ");
          RxArray[2]=((char) MIDI.getData2());
          Serial.println(RxArray[2],HEX);
            usbMIDI.send(RxArray[0]&0xF0,RxArray[1],RxArray[2],Channel,0);
          usbMIDI.send(RxArray[0]&0xF0,RxArray[1],RxArray[2],Channel,1);
          if((RxArray[0]&0xF0)==0xB0 && RxArray[1]==0x7B) {Serial.println("All Notes Off");for (int i=0;i<=15;i++) CurrentState[i]=0;}
      }
      if(RxArray[0]==0xF0){
        Serial.println("Sysex");
      const byte *data = MIDI.getSysExArray();
      for (int i=0;i<=MIDI.getSysExArrayLength();i++){
        RxArray[i]=data[i];
        Serial.print(data[i],HEX);Serial.print(":");
      }
     Serial.println("");
            myHeaderFound=true;
            for (int j=0;j<HeaderLength;j++){
              if (RxArray[j]!=HeaderValue[j]) myHeaderFound=false;
//              Serial.print(j,HEX);Serial.print(RxArray[j],HEX);Serial.print(HeaderValue[j],HEX);Serial.println(myHeaderFound);
            }
            if(myHeaderFound){
             Serial.print("My Header Found ");
             for (int j=HeaderLength; j<MIDI.getSysExArrayLength()-2; j=j+2){
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
      }
      
    }//if command




  
  LoopCounter++;
 
}//loop  
  

/*void SendSerialByte(int SendByte){
  while(MIDI.availableForWrite()==0){}
  MIDI.write((char) SendByte);
}
*/
  


void myNoteOn(byte channel, byte note, byte velocity) {
  Serial.print("ExternalNote On, ch=");
  Serial.print(channel, DEC);
  Serial.print(", note=");
  Serial.print(note, DEC);
  Serial.print(", velocity=");
  Serial.println(velocity, DEC);
//  MIDI.write((unsigned char) (0x90)|((channel)&0xF)); 
//  MIDI.write((unsigned char) note);
//  MIDI.write((unsigned char) velocity);

}



void myNoteOff(byte channel, byte note, byte velocity)
{
  Serial.print("External Note Off, ch=");
  Serial.print(channel, DEC);
  Serial.print(", note=");
  Serial.print(note, DEC);
  Serial.print(", velocity=");
  Serial.println(velocity, DEC);
//  MIDI.write((unsigned char) (0x80)|((channel)&0xF)); 
//  MIDI.write((unsigned char) note);
//  MIDI.write((unsigned char) velocity);


}
  
 void MIDI13Byte(int Command,int Data1,int Data2, int Channel){
  Serial.println("Writing to External Serial port");
//  MIDI.write((unsigned char) (Command&0xF0)|((Channel)&0xF)); 
//  MIDI.write((unsigned char) Data1);
//  MIDI.write((unsigned char) Data2);
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
