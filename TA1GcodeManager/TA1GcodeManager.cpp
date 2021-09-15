#include "TA1GcodeManager.h"

//================class Command===========================

//================class CommandManager====================

void CommandManager::readSerial()
{
  static float lastG = 0;
  static float lastX = 0;
  static float lastY = 0;
  static float lastZ = 0;
  static float lastF = 0;
  static float X=0, Y=0, Z=0, F=0, G=0, S=0, blank=0, *current, point=0, mult=1.0;
  static int wasG=0, wasX=0, wasY=0, wasZ=0, wasF=0;
  static int wasS = 0;
  static int was_command = 0, command_finished = 0, terminated = 0;

  static unsigned int start_micros;

       if(Serial.available() > 0 && !command_finished)
       {
				incomingByte = Serial.read();

				//if(incomingByte == '?') needSendStatus=1;

                if (incomingByte !='\n' && incomingByte != ';') {
                  terminated = 0;
                }


                if(incomingByte>='0' && incomingByte<='9')
                {

                  (*current)*=10;
                  (*current)+=(float)(incomingByte-'0');
                  point*=10.0;
                }
                else if(incomingByte == '-')
                {
                  mult=-1.0;
                }
                else if (incomingByte == '.' || incomingByte == ',')
                {
                  point = 1.0;
				}
				else
                {
                  if(point>0)
                      (*current)/=point;
                    point = .0;
					(*current)*=mult;
					mult=1.0;

					current=&blank;
					(*current)=0;

				  if(incomingByte == 'x' || incomingByte == 'X')
				  {
                  current = &X;
                  point = 0;
                  wasX=1;
                  }
                  else if (incomingByte == 'y' || incomingByte == 'Y')
                  {
                  current = &Y;
                  point = 0;
                  wasY=1;
                  }
                  else if (incomingByte == 'z' || incomingByte == 'Z')
                  {
                  current = &Z;
                  point = 0;
                  wasZ=1;
                  }
                  else if (incomingByte == 'f' || incomingByte == 'F')
                  {
                  current = &F;
                  point = 0;
                  wasF=1;
                  }
                  else if (incomingByte == 'g' || incomingByte == 'G')
                  {
                  current = &G;
                  point = 0;
                  was_command = 1;
                  wasG=1;
                  //start_micros = millis();
                  }
                   else if (incomingByte == 's' || incomingByte == 'S')
                  {
                  current = &S;
                  point = 0;
                  wasS = 1;
                  }
                  else if (incomingByte == '\n' || incomingByte == ';')
                  {
                      if(!terminated) {
                        command_finished = 1;
                      }
                      terminated = 1;
                     // was_command = 0;
                  }
                }
       }

	   if(!command_finished) return;

       if(point>0)
          (*current)/=point;
        (*current)*=mult;

       if(wasG) lastG = G;
       else G = lastG;
       if(wasX) lastX = X;
       else X = lastX;
       if(wasY) lastY = Y;
       else Y = lastY;
       if(wasZ) lastZ = Z;
       else Z = lastZ;
       if(wasF) lastF = F;
       else F = lastF;

	   scheduleCommand(G, X, Y, Z, F);

       if(wasS)
       {
           router.setInstrumentPower((int)S);
	   }

	   command_finished = 0;
	   X=0; Y=0; Z=0; F=0; G=0; S=0; blank=0; point=0; mult=1.0;
       wasG=0; wasX=0; wasY=0; wasZ=0; wasF=0;
       wasS = 0;
       was_command = 0; command_finished = 0;
	   current=&blank;
}


int cCommandManager::canScheduleCommand() {
	if(nCommands<C_BUFFER_LENGTH) return 1;
    return 0;
}

void CommandManager::scheduleCommand(float G, float X, float Y, float Z, float F) {
  c_buffer[nCommands].setValues(G, X, Y, Z, F);
  nCommands++;
  if(nCommands<C_BUFFER_LENGTH)
  {
	Serial.write("K");
  }
}

int CommandManager::canTakeCommand() {
	if(nCommands<=0) return 0;
	return 1;
}

Command CommandManager::takeCommand() {
	nCommands--;

	Command retCom = c_buffer[0];

	for(int i=0; i<nCommands; i++) {
		c_buffer[i].G = c_buffer[i+1].G;
		c_buffer[i].X = c_buffer[i+1].X;
		c_buffer[i].Y = c_buffer[i+1].Y;
		c_buffer[i].Z = c_buffer[i+1].Z;
		c_buffer[i].F = c_buffer[i+1].F;
	}

    return retCom;
}
