//###################################################################
//###################################################################
// Rs485 einlesen eines Bytes 19200 8 n 1

unsigned char readbyte()
{
  unsigned char b;
  //char m[256];
  int timestamp;
  int wait = 100000; // x * 10ms  warte nur 100ms

#define MAXCOUNT 1000000000

  if ((waitcount > MAXCOUNT) || (bytecount > MAXCOUNT))
  {
    waitcount = 0;
    bytecount = 0;
    longwaitcount = 0;
  }

  while (1)
  {
    if (RS485Serial.available())  //Look for data from other Arduino
    {
      bytecount++;
      b = RS485Serial.read();  // Read received byte

      //sprintf(m, "[%d/%d]:%d", bytecount,waitcount, (int) b);
      //client.publish("readbyte", m);
      //delay(10);
      return (b);
    }
    else
    { // kein Input , mx timeout sekunde warten

      waitcount++;
      wait--;

      if ( wait == 0)
      {
        longwaitcount++;
        //sprintf(m, "[%d/%d] -", bytecount,waitcount);
        //client.publish("readbyte", m);
        return (0);
      }

    }
  }
}



/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// ein bestimmtes Bit auslesen
int getbit(unsigned char *data, int nOffset, int nBit)
{
  int nValue;
  nValue = (data[nOffset] >> nBit) & 1;
  return ( nValue);
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// untersch. Messwerte auslesen
double getval2(unsigned char *anData, int nOffset, int nLen, double fFactor, int bSigned)
{
  int   nValue = 0;
  int nI;
  for (nI = 0; nI < nLen; nI++)
  {
    nValue += anData[nOffset + nI] << ((nLen - nI - 1) * 8);

    // wenn val > 2^15 val -= 2^16
    if (bSigned && (nValue > (1 << (nLen * 8 - 1))))
    {
      nValue -= (1 << (nLen * 8));
    }
  }
  return (nValue * fFactor);
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// Hilfsfunktion für Prüfsumme
int CrcAdd(int nCrc, unsigned char nByte)
{
  nCrc = (((nCrc << 1) | ( nCrc >> 7)) & 0xFF);
  nCrc += nByte;
  if (nCrc > 255)
  {
    nCrc -= 255;
  }
  return (nCrc);
}



// Read RS 484 Frame
int readframe(unsigned char anData[],int &nID,int &nDataLen)
{
  int nState, bRxFinished;
  unsigned char nType, nLen, nCounter, nChecksum;
  unsigned char nX;
  int nCrc;  
    nState = STATE_WAIT_FOR_HEADER;
    bRxFinished = FALSE;

    nLen = 0; nCounter = 0; nType = 0; nID = 0; nChecksum = 0; nDataLen = 0; nCrc = 0;
    for (int i = 0; i < 256; i++) anData[i] = 0;

    while (bRxFinished == FALSE)
    {
      nX = readbyte();                           // # read one byte

      if ((nState == STATE_WAIT_FOR_HEADER) && (nX == 2))
      {
        nState = STATE_READ_MSG;             // # header found
        // nType = MSG_TYPE_CTRL;             //          # -> CtrlMessage
      }
      else
      {
        if (nState == STATE_READ_MSG)
        {
          if (nX == 0)                        // # header invalid -> start again
          {
            nState = STATE_WAIT_FOR_HEADER;
            nType = MSG_TYPE_CTRL;
          }
          if (nX == 2)                       //# extended header -> SenseMessage
          {
            nState = STATE_READ_MSG;
            nType = MSG_TYPE_SENSE;
          }
          if ((nX != 0) && (nX != 2))
          {
            nLen = nX;                   //     # current byte: Message Length
            nID = readbyte();            //     # next byte: Message ID
            nCounter = readbyte();       //     # next byte: Message Counter
            nDataLen = nLen - 4 - 1;     //     Data Length = Message Length without the header and checksum

            if ((nDataLen >= 0) && (nDataLen < 256))
              for (int i = 0; (i < nDataLen) ; i++)
              {
                anData[i] = readbyte();
                if ( anData[i] == 2)
                {
                  nChecksum = readbyte();        //      # "2" in data stream is followed by "0" ...
                }
              }
            nChecksum = readbyte();
            bRxFinished = TRUE;
          }
        }
      }
    }

  // kleine Pause um Stromverbrauch zu reduzieren
  // 3 ms nach einem Frame sollte kein Problem sein
  
  delay(5);


  // Prüfsumme berechnen
  nCrc = 0x02;
  nCrc = CrcAdd(nCrc, nLen);
  nCrc = CrcAdd(nCrc, nID);
  nCrc = CrcAdd(nCrc, nCounter);
  for (int i = 0; i < nDataLen; i++)
    nCrc = CrcAdd(nCrc, anData[i]);

    if (nChecksum == nCrc) return 1; 
    else return 0;
 
}



//####################################################################
//####################################################################

// Reconnected den MQTT, wenn er nicht mehr verbunden ist

void mqttreconnect() {
  int i;
  i = 5;
  // loopcount until we're reconnected
  


  while (i-- && !client.connected()) {
    //Serial.print("Attempting MQTT connection...");
   

    if (client.connect(MQNAME)) {
      // Serial.println("connected");
      client.subscribe(INTOPIC);
    } else {
      Serial.print("Attempting MQTT connection...");
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// liefert ne 1 wenn die beiden char Blöcke unterschiedlich sind

bool  messne(unsigned char * a, unsigned char * b, int sz)
{
  int i;

  for (i = 0; i < sz; i++)
    if (a[i] != b[i]) return (1);

  return (0);
}

// konvertiert einen int in einen binärstring

char *inttobin(int n) {
  // determine the number of bits needed ("sizeof" returns bytes)
  int nbits = 8; // sizeof(n) * 8;
  char *s = (char *) malloc(nbits + 1); // +1 for '\0' terminator
  s[nbits] = '\0';

  // forcing evaluation as an unsigned value prevents complications
  // with negative numbers at the left-most bit
  unsigned int u = *(unsigned int*)&n;

  int i;
  unsigned int mask = 1 << (nbits - 1); // fill in values right-to-left
  for (i = 0; i < nbits; i++, mask >>= 1)
    s[i] = ((u & mask) != 0) + '0';

  return s;
}





// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}



// Blink n mal die Webmod onbload LED 

void blink(int n)
{

#define ledPin 2 // led Pin
pinMode(ledPin, OUTPUT);
// Blink
#define INTERVAL 500

while (n-->0)
{
  digitalWrite(ledPin, LOW);
  delay(INTERVAL);
  digitalWrite(ledPin, HIGH);
  delay(INTERVAL);
}
}

void relais(int n)
{
// https://chewett.co.uk/blog/1066/pin-numbering-for-wemos-d1-mini-esp8266/
#define RELAIS 5 // d1 pin
pinMode(RELAIS, OUTPUT);

if (n==1) 
	{
    digitalWrite(RELAIS, HIGH);
	Serial.println("Relais an ");
	}
	else
	{
	
    digitalWrite(RELAIS, LOW);
	Serial.println("Relais aus ");

	}
  
}

//##################################################
