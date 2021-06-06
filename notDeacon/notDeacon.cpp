/**
    @file   notDeacon.cpp
    @author Karsten Schlachter
    @brief  sdr-based ble-beacon test based on basicTx.cpp from Lime Microsystems
 */
#include <iostream>
#include <chrono>
#include <math.h>
#include "lime/LimeSuite.h"
#include <unistd.h>
#include <string.h>

using namespace std;


lms_device_t* device = NULL;

int error()
{
    if (device != NULL)
  	LMS_Close(device);
    exit(-1);
}

void swapbits(uint8_t* data,int len)
{
	uint8_t buff = 0;
	
	for(int byteindex = 0; byteindex < len; byteindex++){
		buff = 0;
		for(int bitindex = 0; bitindex < 8; bitindex++)
		{
			buff |= ((data[byteindex] & (1 <<bitindex))>>bitindex)<<(7-bitindex);
		}

	data[byteindex]=buff;

	}
}


//crc array 3 bytes (24bit) fuer advertising mit 0x555555 fuellen!
void deacon_crc(const uint8_t* data,uint16_t length,uint8_t* crc){
	//crc polynom: x24 + x10 + x9 + x6 + x4 + x3 + x + 1
	const uint8_t poly[] = {0,0b00000110,0b01011011};
	uint8_t carry = 0;
	uint8_t din = 0;
	uint8_t x24 = 0;
	
	//eingangsbytes iterieren
	for (int i=0; i < length; i++)
	{
		//eingangsbits iterieren
		for(int j=0;j < 8; j++)
		{
			//carry zwischenspeichern
			carry = (crc[0] & 0b10000000) >> 7;//bitmaske fuer fkt ueberflussig
			//naechtes bit einlesen
			din = ((data[i] & (1 << (j) )) >> (j));
			//xor fuer eingang durchfuehren
			x24 = carry^din;
			
			//register weitershiften und dabei daten aus vorangehenden bytes uebernehmen
			crc[0] = crc[0] << 1;
			crc[0] = crc[0] | ((crc[1] & 0b10000000) >> 7);
			crc[1] = crc[1] << 1;
			crc[1] = crc[1] | ((crc[2] & 0b10000000) >> 7);
			crc[2] = crc[2] << 1;	
			
			//das letzte bit geht ueber x24 mit bitmaske ein		
			crc[2] = crc[2] ^ (x24 * poly[2]);
			crc[1] = crc[1] ^ (x24 * poly[1]);
		}		
}
	std::cout<< "crc " << std::hex << (int)crc[0] << std::hex << (int)crc[1] 
<<std::hex<<(int)crc[2]<< std::endl;

}


void deacon_dowhitening( uint8_t* data,uint16_t length,uint8_t channel_index){
	//whitening polynom: x7+x4+1
	const uint8_t  poly = 0b00010000;
	
	uint8_t reg = 1; // pos 0 immer 1
	swapbits(&channel_index,1);
	reg = reg | (channel_index >> 1);
	
	//eingangsbytes iterieren	
	for (int i = 0 ; i < length; i++)
	{
		//eingangsbits iterieren
		for (int j = 0 ; j < 8 ; j++)
		{				
			//register nach bit 7 rotieren
			reg = reg << 1;
			reg = reg | ( reg >> 7);
			//dabei ueberhang abschneiden
			reg = reg & 0b01111111;
			
			//polygon anwenden
			reg = reg ^ (poly *(reg & 1));
			
			//datenbit verarbeiten
			//relevantes bit befindet sich nach rotation an pos 1
				data[i] = data[i] ^ ((reg & 1) << (j));
						
		}		
	}	
}

	void deacon_geniqsamples(uint8_t* pkg_data, int pkg_len, float* tx_buffer,int sample_rate)
	{
		const double basemod_freq = 500e3; 
		const double pi = M_PI;
		
		double w = 2 * pi * basemod_freq ;
		int samples_per_symbol = sample_rate / 1e6;//ble uebertragungsrate 1Mbit/s
		
		int sampleindex = 0;
		double t = 0;
		int factor = 2;
		
		for (int byteindex = 0; byteindex < pkg_len; byteindex++)
		{
			for (int bitindex =0; bitindex < 8; bitindex++)
			{
				
				if((pkg_data[byteindex] & (1<<(bitindex))) != 0)
				{
					factor = 2;
					std::cout << "1";
				}else{
					factor = 1;
					std::cout << "0";
				}
				
				//iqsamples fuer jeweiliges bit generieren
				for (int rsampleindex=0; rsampleindex < samples_per_symbol; rsampleindex++)
				{
					t += w * factor / sample_rate;
								  
					tx_buffer[2*sampleindex] = cos(t);// "*-1" um runter statt hochzumischen
					tx_buffer[2*sampleindex+1] =sin(t);
					sampleindex++;
				}
			}
			std::cout << "\n";
			
		}
	}


#define ADV_TEXT_MAX_SIZE 20

struct adPayload
{
	uint8_t ad_address[6];
	uint8_t ad_data[ADV_TEXT_MAX_SIZE+2];/*enthaelt neben text je byte fuer laenge + datentyp*/
}__attribute__((packed));

struct adPDU
{
	uint8_t header[2];
	adPayload payload;

}__attribute__((packed));

struct Paket
{
	uint8_t preamble;
	uint8_t access_address[4];

	adPDU pdu;
	uint8_t crc[3];
}__attribute__((packed));



int main(int argc, char** argv)
{
    const double frequency = 2401.25e6;  //center frequency to x MHz
    const double sample_rate = 40e6;    //sample rate to x MHz
	const int samples_per_symbol = sample_rate / 1e6;//ble uebertragungsrate =  1Mb/s
    
    
	Paket paket={0};

	//preamble muss gem vol6 2.1.1 mit bitwechsel yu adresse uebergehen	
	paket.preamble = 0b01010101;
	
	//adresse fuer advertising broadcast gem vol6 2.1.2 in bt core spec 5.0
	//adv broadcast adresse: 0x8E89BED6 (0b10001110100010011011111011010110)
	paket.access_address[3]=0x8E;
	paket.access_address[2]=0x89;
	paket.access_address[1]=0xBE;
	paket.access_address[0]=0xD6;
	
	//pdu header v6 2.3
   //pdutype adv_nonconn_ind 0b0010 (adv allgemein waere 0b0000)
   //tx add = 1 (random) gem corespec 2.3.1.3
   //keine angabe zu chesel rx add -> reserved -> 0 lassen
   //rfu = reserved for future use
	paket.pdu.header[0] = (0b0010 ) | (1 << 6);

	//payloadlaenge im zweiten teil des headers
	paket.pdu.header[1]=sizeof(paket.pdu.payload);	


	//devaddress (random)
	//vol6 1.3.2.1: die beiden most signifcant muessen 1 sein, im rest min 1 mal 0 u 1 mal 1 
	paket.pdu.payload.ad_address[0]=0b11000011;
	paket.pdu.payload.ad_address[1]=0b11111111;
	paket.pdu.payload.ad_address[2]=0b11101110;
	paket.pdu.payload.ad_address[3]=0b11101110;
	paket.pdu.payload.ad_address[4]=0b01010101;
	paket.pdu.payload.ad_address[5]=0b11000011;

   
	paket.pdu.payload.ad_data[0] = ADV_TEXT_MAX_SIZE + 1;//laenge daten inkl adtyp
	// complete local name = 0x09 (assigned numbers, bt website)
    paket.pdu.payload.ad_data[1] = 0x09;//addata typ = local name
		
	//text ab index 2
	strcpy((char*) &paket.pdu.payload.ad_data[2],"test!");

	//text als parameter uebergeben?
	if(argc > 1)
	{	
		int len = strlen(argv[1]);
		//passt der text in den datenbereich? sonst abschneiden
		if (len > (sizeof(paket.pdu.payload.ad_data) - 2))
		{
			len = sizeof(paket.pdu.payload.ad_data) - 2;
		}
		memcpy(&paket.pdu.payload.ad_data[2],argv[1],len);
	}


	//crc preset fuer advertising: 0x555555 (spec v6 3.1.1)
	paket.crc[0] = 0x55;
	paket.crc[1] = 0x55;
	paket.crc[2] = 0x55;
		
	deacon_crc((uint8_t*) &paket.pdu,sizeof(paket.pdu),(uint8_t*) paket.crc);	
	
	//!!crc wird msb first uebertragen! (spec v6 1.2)	
	//deswegen reihenfolge vor whitening und uebertrage umkehren
	swapbits(paket.crc,sizeof(paket.crc));
	

	cout << "pdu size: "<<sizeof(paket.pdu)<<"\n";

	
	//whitening
	//initialwert abhaengig von channel
	deacon_dowhitening((uint8_t*) &paket.pdu,(sizeof(paket.pdu)+3), 37);


	int pkgbitcount=(sizeof(paket)*8);

	//buffer gross genug fuer alle samples zu paket dimensionieren
	int buffer_size = pkgbitcount * samples_per_symbol;
	float tx_buffer[2*buffer_size];//mal 2 da iq samples
	std::cout << "buffer size: " << buffer_size << "\n";
	

	//samples zum versenden aus daten generieren
	deacon_geniqsamples((uint8_t*) &paket,sizeof(paket),tx_buffer,sample_rate);

	
	//========================================================
	//initialiserungskram aus basicTX.cpp (LimeSDK Beispiel)

	//Find devices
    int n;
    lms_info_str_t list[8]; //should be large enough to hold all detected devices
    if ((n = LMS_GetDeviceList(list)) < 0) //NULL can be passed to only get number of devices
        error();

    cout << "Devices found: " << n << endl; //print number of devices
    if (n < 1)
        return -1;

    //open the first device
    if (LMS_Open(&device, list[0], NULL))
        error();

    //Initialize device with default configuration
    //Do not use if you want to keep existing configuration
    //Use LMS_LoadConfig(device, "/path/to/file.ini") to load config from INI
    if (LMS_Init(device)!=0)
        error();

    //Enable TX channel,Channels are numbered starting at 0
    if (LMS_EnableChannel(device, LMS_CH_TX, 0, true)!=0)
        error();

    //Set sample rate
    if (LMS_SetSampleRate(device, sample_rate, 0)!=0)
        error();
    cout << "Sample rate: " << sample_rate/1e6 << " MHz" << endl;

    //Set center frequency
    if (LMS_SetLOFrequency(device,LMS_CH_TX, 0, frequency)!=0)
        error();
    cout << "Center frequency: " << frequency/1e6 << " MHz" << endl;

    //select TX1_1 antenna
	//tx2-antennen sind die fuer oberen frequenzbereich
    if (LMS_SetAntenna(device, LMS_CH_TX, 0, LMS_PATH_TX2)!=0)
        error();

    //set TX gain
    if (LMS_SetNormalizedGain(device, LMS_CH_TX, 0, 0.8) != 0)
        error();

    //calibrate Tx, continue on failure
    LMS_Calibrate(device, LMS_CH_TX, 0, sample_rate, 0);
    
    //Streaming Setup
    
    lms_stream_t tx_stream;                 //stream structure
    tx_stream.channel = 0;                  //channel number
    tx_stream.fifoSize = 2 * pkgbitcount * sample_rate / 1e6;//256*1024;          //fifo size in samples
    tx_stream.throughputVsLatency = 0.5;    //0 min latency, 1 max throughput
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32; //floating point samples
    tx_stream.isTx = true;                  //TX channel
    LMS_SetupStream(device, &tx_stream);
	

    LMS_StartStream(&tx_stream);         //Start streaming
    //Streaming
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = t1;


	int send_cnt = buffer_size;

	cout << "sendcount: "<<send_cnt<<endl;
	

   while (true)
    {

		//sampledaten fuer ble paket limesdr senden
		int ret = LMS_SendStream(&tx_stream,tx_buffer, send_cnt, nullptr, 1000);
			
		//erfolg pruefen
		if (ret != send_cnt)
		{
			cout << "error: samples sent: " << ret << "/" << send_cnt << endl;
		}
		
		if (chrono::high_resolution_clock::now() - t2 > chrono::seconds(1))
		{
			t2 = chrono::high_resolution_clock::now();
			lms_stream_status_t status;
			LMS_GetStreamStatus(&tx_stream, &status);  //Get stream status
			//cout << "TX data rate: " << status.linkRate / 1e6 << " MB/s\n"; //link data rate
			cout << "*"<<flush;//lebenszeichen von sich geben
		}

		//anschliessend fuer ein advertising interval warten
		//adv-interval = 20ms-10s + 0-10ms rnd gegen kollision        
		usleep(100000);	

    }
	
	//========================================================
	//cleanup aus basicTx.cpp
    //Stop streaming
    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(device, &tx_stream);

    //Disable TX channel
    if (LMS_EnableChannel(device, LMS_CH_TX, 0, false)!=0)
        error();

    //Close device
    if (LMS_Close(device)==0)
        cout << "Closed" << endl;
    return 0;
}
