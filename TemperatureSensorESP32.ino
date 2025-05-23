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
#include <Adafruit_PCF8574.h>

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
static const uint32_t SampleCheckMax = 100000;
static const uint8_t PCF8574Address = 0x20;
static const uint8_t IOExpanderChannel = 3;
static const uint8_t FanPin = 4;
static const uint8_t HeatPelterPin0 = 1;
static const uint8_t HeatPelterPin1 = 3;
static const uint8_t CoolPelterPin0 = 0;
static const uint8_t CoolPelterPin1 = 2;

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
uint8_t IOExpanderPins = 0;

uint32_t PeltierCycleMaxTime = 100000;
uint32_t PeltierLastTime = 0;
uint32_t PeltierActiveTimeMax = 0;
uint32_t PeltierStartTime = 0;
float PeltierPowerSetting = 0.0;
bool PeltierActive = false;
bool PeltierCooling = false;
bool PeltierCoolingNeeded = false;

struct MeasurementStatistics
{
	float LastReadingTime;
	float LastFlushTime;
	float LastReading;
	bool New;
	double Mean;
	double STD;
	double Min;
	double Max;
	double Slope;
	double Intercept;
	double Correlation;
	double Timespan;
	RunningRegression Stats;
};

Adafruit_PCF8574 IOExpander;

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
	Serial.print(Label);
	Serial.print(",");
	Serial.print(Index);
	Serial.print(",");
	Serial.print(Target->Mean,4);
	Serial.print(",");
	Serial.print(Target->STD,4);
	Serial.print(",");
	Serial.print(Target->Min,4);
	Serial.print(",");
	Serial.print(Target->Max,4);
	Serial.print(",");
	Serial.print(Target->Slope,4);
	Serial.print(",");
	Serial.print(Target->Intercept,4);
	Serial.print(",");
	Serial.print(Target->Correlation,4);
	Serial.print(",");
	Serial.print(Target->Timespan);
	Serial.print(",");
	Serial.print(Target->New);
	Serial.print("\n");
	Target->New = false;
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
			Target->New = false;
			Target->Mean = Target->Stats.MeanY();
			Target->STD = Target->Stats.StandardDeviationY();
			Target->Min = Target->Stats.MinY();
			Target->Max = Target->Stats.MaxY();
			Target->Slope = Target->Stats.Slope();
			Target->Intercept = Target->Stats.Intercept();
			Target->Correlation = Target->Stats.Correlation();
			Target->Timespan = Target->Stats.MaxX() - Target->Stats.MinX();
			Target->Stats.Clear();
			Target->LastFlushTime = (float)(millis())/1000.0;
			//PrintMeasurementStatistics(Target, Index, Label);
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

void SetPeltierOff()
{
	IOExpanderPins = 0;
	I2CMultiplexer.selectChannel(IOExpanderChannel);
	IOExpander.digitalWriteByte(IOExpanderPins);
	PeltierActive = false;
}

void SetPeltierCooling()
{
	IOExpanderPins = 0;
	bitWrite(IOExpanderPins, FanPin, 1);
	bitWrite(IOExpanderPins, HeatPelterPin0, 0);
	bitWrite(IOExpanderPins, HeatPelterPin1, 0);
	bitWrite(IOExpanderPins, CoolPelterPin0, 1);
	bitWrite(IOExpanderPins, CoolPelterPin1, 1);
	I2CMultiplexer.selectChannel(IOExpanderChannel);
	IOExpander.digitalWriteByte(IOExpanderPins);
	PeltierActive = true;
	PeltierCooling = true;
}

void SetPeltierHeating()
{
	IOExpanderPins = 0;
	bitWrite(IOExpanderPins, FanPin, 1);
	bitWrite(IOExpanderPins, HeatPelterPin0, 1);
	bitWrite(IOExpanderPins, HeatPelterPin1, 1);
	bitWrite(IOExpanderPins, CoolPelterPin0, 0);
	bitWrite(IOExpanderPins, CoolPelterPin1, 0);
	I2CMultiplexer.selectChannel(IOExpanderChannel);
	IOExpander.digitalWriteByte(IOExpanderPins);
	PeltierActive = true;
	PeltierCooling = false;
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

void SetPeltierPower(float PeltierPowerToSet)
{
	PeltierPowerSetting = constrain(PeltierPowerToSet,-1.0,1.0);
	if (PeltierPowerSetting == 0.0)
	{
		SetPeltierOff();
		PeltierActiveTimeMax = 0;
	}
	else
	{
		PeltierCoolingNeeded = PeltierPowerSetting > 0.0;
		float DutyCycle = abs(PeltierPowerSetting);
		PeltierActiveTimeMax = (uint32_t)(DutyCycle * (float)(PeltierCycleMaxTime));
		if (PeltierActiveTimeMax == 0)
		{
			SetPeltierOff();
			PeltierPowerSetting = 0.0;
		}
		if (PeltierActiveTimeMax > PeltierCycleMaxTime)
		{
			PeltierActiveTimeMax = PeltierCycleMaxTime;
		}
		//Serial.print("Duty ");
		//Serial.print(PeltierActiveTimeMax);
		//Serial.print(" of ");
		//Serial.print(PeltierCycleMaxTime);
		//Serial.print("\n");
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

void CommandPeltierOff(TextCommandParser::Argument *args, char *response)
{
	SetPeltierPower(0.0);
	SetPeltierOff();
	CommandResponse.clear();
	CommandResponse.print("Peltier off.");
}

void CommandPeltierCool(TextCommandParser::Argument *args, char *response)
{
	SetPeltierPower(-1.0);
	SetPeltierCooling();
	CommandResponse.clear();
	CommandResponse.print("Peltier cooling.");
}

void CommandPeltierHeat(TextCommandParser::Argument *args, char *response)
{
	SetPeltierPower(1.0);
	SetPeltierHeating();
	CommandResponse.clear();
	CommandResponse.print("Peltier heating.");
}

void CommandSetPeltierPower(TextCommandParser::Argument *args, char *response)
{
	double PowerToSet = args[0].asDouble;
	PowerToSet = constrain(PowerToSet,-1.0,1.0);
	SetPeltierPower( (float)(PowerToSet) );
	CommandResponse.clear();
	CommandResponse.print("Peltier power set to ");
	CommandResponse.print(PeltierPowerSetting);
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
	Parser.registerCommand("PeltierOff", "", &CommandPeltierOff);
	Parser.registerCommand("PeltierCool", "", &CommandPeltierCool);
	Parser.registerCommand("PeltierHeat", "", &CommandPeltierHeat);
	Parser.registerCommand("PeltierPower", "d", &CommandSetPeltierPower);
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

void CheckPeltier()
{
	if ( (micros() - PeltierLastTime) > PeltierCycleMaxTime)
	{
		PeltierLastTime = micros();
		if ( (PeltierActiveTimeMax > 0) & (!PeltierActive) )
		{
			if (PeltierCoolingNeeded)
			{
				SetPeltierCooling();
			}
			else
			{
				SetPeltierHeating();
			}
			PeltierStartTime = micros();
		}
	}
	if ( (PeltierActive) & (PeltierActiveTimeMax < PeltierCycleMaxTime) )
	{
		if ( (micros() - PeltierStartTime) > PeltierActiveTimeMax)
		{
			PeltierStartTime = micros();
			SetPeltierOff();
		}
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
	Serial.print("Setting up IO expander.\n");
	I2CMultiplexer.selectChannel(IOExpanderChannel);
	IOExpander.begin(PCF8574Address, &Wire);
	IOExpander.pinMode(FanPin,OUTPUT);
	IOExpander.pinMode(HeatPelterPin0,OUTPUT);
	IOExpander.pinMode(HeatPelterPin1,OUTPUT);
	IOExpander.pinMode(CoolPelterPin0,OUTPUT);
	IOExpander.pinMode(CoolPelterPin1,OUTPUT);
	IOExpander.digitalWriteByte(IOExpanderPins);
	Serial.print("IO expander setup complete.\n");
	SetStatusLED(false);
	Serial.print("Boot finished.\n");
}

void loop()
{
	CheckUSBSerial();
	CheckSamples();
	CheckPeltier();
}