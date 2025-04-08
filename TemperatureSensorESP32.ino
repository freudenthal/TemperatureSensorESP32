#include <Wire.h>
#include <CommandParser.h>
#include <TCA9548.h>
#include <PrintCharArray.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TMP117.h>
#include <RunningStats.h>
#include <RunningRegression.h>
#include <Adafruit_Sensor.h>

typedef CommandParser<16, 4, 32, 128, 255> TextCommandParser;

static const uint32_t USBSerialRate = 115200;
static const uint32_t I2CRate = 100000;
static const uint8_t SDAPin = 22;
static const uint8_t SCLPin = 20;
static const uint8_t LEDPin = 13;
static const uint8_t I2CPowerPin = 2;
static const uint8_t NeoPixelPin = 0;
static const uint8_t TCA9548Address = 0x70;
static const uint8_t I2CBusCount = 8;
static const uint16_t LineBufferCount = 256;
static const uint32_t SerialCheckMax = 100000;
static const uint32_t BME280Address = 0x77;
static const uint8_t BMECount = 3;
static const uint8_t BMEBus[] = {0,1,2};
static const uint32_t TMP117Address = 0x48;
static const uint8_t TMPCount = 2;
static const uint8_t TMPBus[] = {1,2};
static const uint32_t SampleCheckMax = 100000;

uint8_t NeoPixelBrightness = 20;

TCA9548 I2CMultiplexer(TCA9548Address);
Adafruit_NeoPixel pixel(1, NeoPixelPin, NEO_GRB + NEO_KHZ800);
char LineBuffer[LineBufferCount];
uint8_t LineBufferReadLength = 0;
char LastChar = '\0';
uint32_t LastSerialCheck = 0;
PrintCharArray CommandResponse(TextCommandParser::MAX_RESPONSE_SIZE);
TextCommandParser Parser;
uint8_t SampleCountMax = 32;
uint32_t LastSampleCheck = 0;

struct MeasurementStatistics
{
	float LastReadingTime;
	float LastFlushTime;
	float LastReading;
	RunningRegression Stats;
};

Adafruit_BME280 BME0;
Adafruit_BME280 BME1;
Adafruit_BME280 BME2;
Adafruit_BME280* BMESet[] = {&BME0, &BME1, &BME2};

Adafruit_Sensor *BME0TempSensor = BME0.getTemperatureSensor();
Adafruit_Sensor *BME1TempSensor = BME1.getTemperatureSensor();
Adafruit_Sensor *BME2TempSensor = BME2.getTemperatureSensor();
Adafruit_Sensor *BMETempSensors[] = {BME0TempSensor,BME1TempSensor,BME2TempSensor};
MeasurementStatistics BME0TempStat;
MeasurementStatistics BME1TempStat;
MeasurementStatistics BME2TempStat;
MeasurementStatistics* BMETempStats[] = {&BME0TempStat,&BME1TempStat,&BME2TempStat};

Adafruit_Sensor *BME0PresSensor = BME0.getPressureSensor();
Adafruit_Sensor *BME1PresSensor = BME1.getPressureSensor();
Adafruit_Sensor *BME2PresSensor = BME2.getPressureSensor();
Adafruit_Sensor *BMEPresSensors[] = {BME0PresSensor,BME1PresSensor,BME2PresSensor};
MeasurementStatistics BME0PresStat;
MeasurementStatistics BME1PresStat;
MeasurementStatistics BME2PresStat;
MeasurementStatistics* BMEPresStats[] = {&BME0PresStat,&BME1PresStat,&BME2PresStat};

Adafruit_Sensor *BME0HumiSensor = BME0.getHumiditySensor();
Adafruit_Sensor *BME1HumiSensor = BME1.getHumiditySensor();
Adafruit_Sensor *BME2HumiSensor = BME2.getHumiditySensor();
Adafruit_Sensor *BMEHumiSensors[] = {BME0HumiSensor,BME1HumiSensor,BME2HumiSensor};
MeasurementStatistics BME0HumiStat;
MeasurementStatistics BME1HumiStat;
MeasurementStatistics BME2HumiStat;
MeasurementStatistics* BMEHumiStats[] = {&BME0HumiStat,&BME1HumiStat,&BME2HumiStat};

Adafruit_TMP117 TMP0;
Adafruit_TMP117 TMP1;
Adafruit_TMP117* TMPSet[] = {&TMP0,&TMP1};
MeasurementStatistics TMP0Stat;
MeasurementStatistics TMP1Stat;
MeasurementStatistics* TMPStats[] = {&TMP0Stat,&TMP1Stat};

void SetNeoPixelI2CPower(bool Enable)
{
	pinMode(LEDPin, OUTPUT);
	digitalWrite(LEDPin, Enable);
}

void SetStatusLED(bool Enable)
{
	pinMode(NeoPixelPin, OUTPUT);
	digitalWrite(NeoPixelPin, Enable);
}

void InitializeNeoPixel()
{
	pixel.begin();
	pixel.setBrightness(NeoPixelBrightness);
	pixel.setPixelColor(0, 0xFFFFFF);
	pixel.show();
}

void SetNeoPixel(uint8_t Red, uint8_t Green, uint8_t Blue)
{
	pixel.setBrightness(NeoPixelBrightness);
	pixel.setPixelColor(0, Red, Green, Blue);
	pixel.show();
}

void PrintMeasurementStatistics(MeasurementStatistics* Target, uint8_t Index, const char* Label)
{
	double TimeSpan  = Target->Stats.MaxX() - Target->Stats.MinX();
	double Correlation = Target->Stats.Correlation();
	Serial.print(Label);
	Serial.print(",");
	Serial.print(Index);
	Serial.print(",");
	Serial.print(Target->Stats.MeanY(),4);
	Serial.print(",");
	Serial.print(Target->Stats.StandardDeviationY(),4);
	Serial.print(",");
	Serial.print(Target->Stats.MinY(),4);
	Serial.print(",");
	Serial.print(Target->Stats.MaxY(),4);
	Serial.print(",");
	Serial.print(Target->Stats.Slope(),4);
	Serial.print(",");
	Serial.print(Target->Stats.Intercept(),4);
	Serial.print(",");
	Serial.print( (Correlation*Correlation) ,4);
	Serial.print(",");
	Serial.print(TimeSpan);
	Serial.print("\n");
}

void UpdateWithNewReading(sensors_event_t SensorEvent, MeasurementStatistics* Target, uint8_t Index, const char* Label)
{
	float TemporaryReading = SensorEvent.temperature; //This doesn't matter, all are a union of float values
	float EventTime = (float)(SensorEvent.timestamp)/1000.0;
	float LastFlushTime = Target->LastFlushTime;
	float LastReading = Target->LastReading;
	float LastReadingTime = Target->LastReadingTime;
	bool ReadingDifferent = TemporaryReading != LastReading;
	float TemporaryTime = EventTime - LastFlushTime;
	bool TimeValid = TemporaryTime > 0.0;
	bool ReadingValid = !isnan(TemporaryReading);
	if (ReadingDifferent && TimeValid && ReadingValid)
	{
		Target->LastReading = TemporaryReading;
		Target->LastReadingTime = TemporaryTime;
		Target->Stats.Push((double)TemporaryTime,(double)TemporaryReading);
		if (Target->Stats.NumDataValues() >= SampleCountMax)
		{
			PrintMeasurementStatistics(Target, Index, Label);
			Target->Stats.Clear();
			Target->LastFlushTime = (float)(millis())/1000.0;
		}
	}
}

void UpdateMeasurementStatistics(const uint8_t Bus[], Adafruit_Sensor* Sensors[], MeasurementStatistics* Stats[], uint8_t Count, const char* Label)
{
	bool Status;
	sensors_event_t SensorEvent;
	uint8_t ActiveChannel;
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<Count; I2CBusIndex++)
	{
		ActiveChannel = Bus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		Status = Sensors[I2CBusIndex]->getEvent(&SensorEvent);
		if (Status)
		{
			UpdateWithNewReading(SensorEvent, Stats[I2CBusIndex], I2CBusIndex, Label);
		}
	}
}

void UpdateMeasurementStatistics(const uint8_t Bus[], Adafruit_TMP117* Sensors[], MeasurementStatistics* Stats[], uint8_t Count, const char* Label)
{
	bool Status;
	sensors_event_t SensorEvent;
	uint8_t ActiveChannel;
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<Count; I2CBusIndex++)
	{
		ActiveChannel = Bus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		Status = Sensors[I2CBusIndex]->getEvent(&SensorEvent);
		if (Status)
		{
			UpdateWithNewReading(SensorEvent, Stats[I2CBusIndex], I2CBusIndex, Label);
		}
	}
}

void ScanAllI2CChannels()
{
	bool Status = false;
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<I2CBusCount; I2CBusIndex++)
	{
		Status = I2CMultiplexer.selectChannel(I2CBusIndex);
		Serial.print("I2C bus index ");
		Serial.print(I2CBusIndex);
		if (Status)
		{
			Serial.print(" active.\n");
		}
		else
		{
			Serial.print(" failed to activate.\n");
		}
		ScanI2C();
	}
}

void ScanI2C()
{
	uint8_t error = 0;
	uint8_t nDevices = 0;
	for(uint8_t address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
			{
			Serial.print("0");
			}
			Serial.print(address,HEX);
			Serial.print(".\n");
			nDevices++;
		}
		else if (error==4)
		{
			Serial.print("Unknown error at address 0x");
			if (address<16)
			{
				Serial.print("0");
			}
			Serial.print(address,HEX);
			Serial.print('\n');
		}
		//else
		//{
		//	Serial.print("Nothing at ");
		//	Serial.print(address);
		//	Serial.print(" with ");
		//	Serial.print(error);
		//	Serial.println(".");
		//}
	}
	if (nDevices == 0)
	{
	Serial.print("No I2C devices found.\n");
	}
	else
	{
	Serial.print("I2C scan done found ");
	Serial.print(nDevices);
	Serial.print(" devices found.\n");
	}
}

void CommandIdentify(TextCommandParser::Argument *args, char *response)
{
	CommandResponse.clear();
	CommandResponse.print("ESP32Temperature");
}

void CommandI2CBusScan(TextCommandParser::Argument *args, char *response)
{
	ScanI2C();
	CommandResponse.clear();
	CommandResponse.print("I2C bus scan starting.");
}

void CommandI2CBusScanAllChannels(TextCommandParser::Argument *args, char *response)
{
	ScanAllI2CChannels();
	//strlcpy(CommandResponse, "I2C scan of all buses starting.", TextCommandParser::MAX_RESPONSE_SIZE);
	CommandResponse.clear();
	CommandResponse.print("I2C scan of all buses starting.");
}

void ClearLineBuffer()
{
	memset(LineBuffer, 0, sizeof(LineBuffer));
	LineBufferReadLength = 0;
	LastChar = '\0';
}

void BuildParser()
{
	ClearLineBuffer();
	Parser.registerCommand("ID", "", &CommandIdentify);
	Parser.registerCommand("I2CScan", "", &CommandI2CBusScan);
	Parser.registerCommand("I2CScanAll", "", &CommandI2CBusScanAllChannels);
}

void ProcessLineBuffer()
{
	LineBuffer[LineBufferReadLength] = '\0';
	Parser.processCommand(LineBuffer, CommandResponse.getBuffer());
	if (CommandResponse.size() > 0)
	{
		CommandResponse.print("\n");
		Serial.write(CommandResponse.getBuffer(),CommandResponse.size());
	}
}

void ParseSerial()
{
	while (Serial.available() > 0)
	{
		char NewChar = Serial.read();
		bool NewCharIsLineEnd = (NewChar == '\n') || (NewChar == ';') || (NewChar == '\r');
		bool NewCharIsBlankOrEnd = ( isspace(NewChar) || NewCharIsLineEnd );
		bool NewCharIsIgnored = (NewChar == '\t') || (NewChar == '\v') || (NewChar == '\f');
		bool NewCharIsDoubleSpace = isspace(NewChar) && isspace(LastChar);
		bool IsLeadingWhiteSpace = (LineBufferReadLength == 0) && NewCharIsBlankOrEnd;
		SetStatusLED(true);
		if ( IsLeadingWhiteSpace || NewCharIsIgnored || NewCharIsDoubleSpace )
		{
			//Do nothing
		}
		else if ( NewCharIsLineEnd )
		{
			if (LineBufferReadLength > 1)
			{
				ProcessLineBuffer();
			}
			ClearLineBuffer();
			SetStatusLED(false);
			break;
		}
		else
		{
			LastChar = NewChar;
			LineBuffer[LineBufferReadLength] = NewChar;
			LineBufferReadLength++;
			if (LineBufferReadLength >= LineBufferCount)
			{
				ClearLineBuffer();
				Serial.println("Input buffer overflow. Input cleared.");
				SetStatusLED(false);
				break;
			}
		}
		SetStatusLED(false);
	}
}

void CheckUSBSerial()
{
	if ( micros() - LastSerialCheck > SerialCheckMax )
	{
		LastSerialCheck = micros();
		ParseSerial();
	}
}

void CheckSamples()
{
	if ( micros() - LastSampleCheck > SampleCheckMax )
	{
		LastSampleCheck = micros();
		SetNeoPixel(255,0,0);
		UpdateMeasurementStatistics(BMEBus, BMETempSensors, BMETempStats, BMECount, "BMETemp");
		SetNeoPixel(0,255,0);
		UpdateMeasurementStatistics(BMEBus, BMEPresSensors, BMEPresStats, BMECount, "BMEPres");
		SetNeoPixel(0,0,255);
		UpdateMeasurementStatistics(BMEBus, BMEHumiSensors, BMEHumiStats, BMECount, "BMEHumi");
		SetNeoPixel(255,0,255);
		UpdateMeasurementStatistics(TMPBus, TMPSet, TMPStats, TMPCount, "TMPTemp");
		SetNeoPixel(0,0,0);
	}
}

void setup()
{
	Serial.begin(USBSerialRate);
	Serial.print("Boot\n");
	SetStatusLED(true);
	Wire.setClock(I2CRate);
	Wire.setPins(SDAPin,SCLPin);
	Wire.begin();
	Serial.print("I2C init complete.\n");
	BuildParser();
	Serial.print("Parser init complete.\n");
	ClearLineBuffer();
	I2CMultiplexer.begin();
	Serial.print("I2C multiplexer init complete.\n");
	InitializeNeoPixel();
	Serial.print("NeoPixel init complete.\n");
	SetNeoPixel(0,0,0);
	Serial.print("NeoPixel setting complete.\n");
	uint8_t ActiveChannel = 0;
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<BMECount; I2CBusIndex++)
	{
		ActiveChannel = BMEBus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		BMESet[I2CBusIndex]->begin(BME280Address,&Wire);
		BMESet[I2CBusIndex]->setSampling(
			Adafruit_BME280::sensor_mode::MODE_NORMAL,
			Adafruit_BME280::sensor_sampling::SAMPLING_X1,
			Adafruit_BME280::sensor_sampling::SAMPLING_X1,
			Adafruit_BME280::sensor_sampling::SAMPLING_X1,
			Adafruit_BME280::sensor_filter::FILTER_OFF,
			Adafruit_BME280::standby_duration::STANDBY_MS_0_5);
	}
	Serial.print("BME280 init complete.\n");
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<TMPCount; I2CBusIndex++)
	{
		ActiveChannel = TMPBus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		TMPSet[I2CBusIndex]->begin(TMP117Address,&Wire);
		TMPSet[I2CBusIndex]->setAveragedSampleCount(tmp117_average_count_t::TMP117_AVERAGE_1X);
		TMPSet[I2CBusIndex]->setMeasurementMode(tmp117_mode_t::TMP117_MODE_CONTINUOUS);
	}
	Serial.print("TMP117 init complete.\n");
	SetStatusLED(false);
}

void loop()
{
	CheckUSBSerial();
	CheckSamples();
}