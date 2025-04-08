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

Adafruit_BME280 BME0;
Adafruit_BME280 BME1;
Adafruit_BME280 BME2;
Adafruit_BME280* BMESet[] = {&BME0, &BME1, &BME2};

Adafruit_Sensor *BME0TempSensor = BME0.getTemperatureSensor();
Adafruit_Sensor *BME1TempSensor = BME1.getTemperatureSensor();
Adafruit_Sensor *BME2TempSensor = BME2.getTemperatureSensor();
Adafruit_Sensor *BMETempSensors[] = {BME0TempSensor,BME1TempSensor,BME2TempSensor};
float BME0TempLastReadingTime = 0.0;
float BME1TempLastReadingTime = 0.0;
float BME2TempLastReadingTime = 0.0;
float* BMETempLastReadingTimes[] = {&BME0TempLastReadingTime, &BME1TempLastReadingTime, &BME2TempLastReadingTime};
float BME0TempLastFlushTime = 0.0;
float BME1TempLastFlushTime = 0.0;
float BME2TempLastFlushTime = 0.0;
float* BMETempLastFlushTimes[] = {&BME0TempLastFlushTime, &BME1TempLastFlushTime, &BME2TempLastFlushTime};
float BME0TempLastReading = 0.0;
float BME1TempLastReading = 0.0;
float BME2TempLastReading = 0.0;
float* BMETempLastReadings[] = {&BME0TempLastReading, &BME1TempLastReading, &BME2TempLastReading};
RunningRegression BME0Temperature;
RunningRegression BME1Temperature;
RunningRegression BME2Temperature;
RunningRegression* BMETemperatures[] = {&BME0Temperature, &BME1Temperature, &BME2Temperature};

Adafruit_Sensor *BME0PresSensor = BME0.getPressureSensor();
Adafruit_Sensor *BME1PresSensor = BME1.getPressureSensor();
Adafruit_Sensor *BME2PresSensor = BME2.getPressureSensor();
Adafruit_Sensor *BMEPresSensors[] = {BME0PresSensor,BME1PresSensor,BME2PresSensor};
float BME0PresLastReadingTime = 0.0;
float BME1PresLastReadingTime = 0.0;
float BME2PresLastReadingTime = 0.0;
float* BMEPresLastReadingTimes[] = {&BME0PresLastReadingTime, &BME1PresLastReadingTime, &BME2PresLastReadingTime};
float BME0PresLastFlushTime = 0.0;
float BME1PresLastFlushTime = 0.0;
float BME2PresLastFlushTime = 0.0;
float* BMEPresLastFlushTimes[] = {&BME0PresLastFlushTime, &BME1PresLastFlushTime, &BME2PresLastFlushTime};
float BME0PresLastReading = 0.0;
float BME1PresLastReading = 0.0;
float BME2PresLastReading = 0.0;
float* BMEPresLastReadings[] = {&BME0PresLastReading, &BME1PresLastReading, &BME2PresLastReading};
RunningRegression BME0Pressure;
RunningRegression BME1Pressure;
RunningRegression BME2Pressure;
RunningRegression* BMEPressures[] = {&BME0Pressure, &BME1Pressure, &BME2Pressure};

Adafruit_Sensor *BME0HumiSensor = BME0.getHumiditySensor();
Adafruit_Sensor *BME1HumiSensor = BME1.getHumiditySensor();
Adafruit_Sensor *BME2HumiSensor = BME2.getHumiditySensor();
Adafruit_Sensor *BMEHumiSensors[] = {BME0HumiSensor,BME1HumiSensor,BME2HumiSensor};
float BME0HumiLastReadingTime = 0.0;
float BME1HumiLastReadingTime = 0.0;
float BME2HumiLastReadingTime = 0.0;
float* BMEHumiLastReadingTimes[] = {&BME0HumiLastReadingTime, &BME1HumiLastReadingTime, &BME2HumiLastReadingTime};
float BME0HumiLastFlushTime = 0.0;
float BME1HumiLastFlushTime = 0.0;
float BME2HumiLastFlushTime = 0.0;
float* BMEHumiLastFlushTimes[] = {&BME0HumiLastFlushTime, &BME1HumiLastFlushTime, &BME2HumiLastFlushTime};
float BME0HumiLastReading = 0.0;
float BME1HumiLastReading = 0.0;
float BME2HumiLastReading = 0.0;
float* BMEHumiLastReadings[] = {&BME0HumiLastReading, &BME1HumiLastReading, &BME2HumiLastReading};
RunningRegression BME0Humidity;
RunningRegression BME1Humidity;
RunningRegression BME2Humidity;
RunningRegression* BMEHumidities[] = {&BME0Humidity, &BME1Humidity, &BME2Humidity};

Adafruit_TMP117 TMP0;
Adafruit_TMP117 TMP1;
Adafruit_TMP117* TMPSet[] = {&TMP0,&TMP1};

float TMP0LastReadingTime = 0.0;
float TMP1LastReadingTime = 0.0;
float* TMPLastReadingTimes[] = {&TMP0LastReadingTime, &TMP1LastReadingTime};
float TMP0LastReading = 0.0;
float TMP1LastReading = 0.0;
float* TMPLastReadings[] = {&TMP0LastReading, &TMP1LastReading};
float TMP0LastFlushTime = 0.0;
float TMP1LastFlushTime = 0.0;
float* TMPLastFlushTimes[] = {&TMP0LastFlushTime, &TMP1LastFlushTime};
RunningRegression TMP0Temperature;
RunningRegression TMP1Temperature;
RunningRegression* TMPTemperatures[] = {&TMP0Temperature, &TMP1Temperature};

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

void UpdateBMETemperature()
{
	bool Status;
	sensors_event_t SensorEvent;
	float TempReading;
	float TempTime;
	float FlushTime;
	double TimeSpan;
	uint8_t ActiveChannel;
	SetNeoPixel(128,0,0);
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<BMECount; I2CBusIndex++)
	{
		ActiveChannel = BMEBus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		Status = BMETempSensors[I2CBusIndex]->getEvent(&SensorEvent);
		if (Status)
		{
			TempReading = SensorEvent.temperature;
			if (TempReading != *BMETempLastReadings[I2CBusIndex])
			{
				FlushTime = (float)((*BMETempLastFlushTimes[I2CBusIndex]))/1000.0;
				TempTime = ((float)(SensorEvent.timestamp)/1000.0) - FlushTime;
				if (TempTime > 0.0)
				{
					*BMETempLastReadings[I2CBusIndex] = TempReading;
					*BMETempLastReadingTimes[I2CBusIndex] = TempTime;
					BMETemperatures[I2CBusIndex]->Push((double)TempTime,(double)TempReading);
					if ( (BMETemperatures[I2CBusIndex]->NumDataValues() >= SampleCountMax) && (TempTime > 0.0) )
					{
						TimeSpan  = BMETemperatures[I2CBusIndex]->MaxX() - BMETemperatures[I2CBusIndex]->MinX();
						Serial.print("BMETemp,");
						Serial.print(I2CBusIndex);
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->MeanY());
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->StandardDeviationY());
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->MinY());
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->MaxY());
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->Slope());
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->Intercept());
						Serial.print(",");
						Serial.print(BMETemperatures[I2CBusIndex]->Correlation());
						Serial.print(",");
						Serial.print(TimeSpan);
						Serial.print("\n");
						BMETemperatures[I2CBusIndex]->Clear();
						*BMETempLastFlushTimes[I2CBusIndex] = millis();
					}
				}
				//else
				//{
				//	Serial.print("BME Temp time error ");
				//	Serial.print(TempTime);
				//	Serial.print("\n");
				//	BMETemperatures[I2CBusIndex]->Clear();
				//	BMETempFlushTime = millis();
				//}
			}
		}
	}
	SetNeoPixel(0,0,0);
}

void UpdateBMEPressure()
{
	bool Status;
	sensors_event_t SensorEvent;
	float TempReading;
	float TempTime;
	double TimeSpan;
	float FlushTime;
	uint8_t ActiveChannel;
	SetNeoPixel(0,128,0);
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<BMECount; I2CBusIndex++)
	{
		ActiveChannel = BMEBus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		Status = BMEPresSensors[I2CBusIndex]->getEvent(&SensorEvent);
		if (Status)
		{
			TempReading = SensorEvent.pressure;
			if (TempReading != *BMEPresLastReadings[I2CBusIndex])
			{
				FlushTime = (float)((*BMEPresLastFlushTimes[I2CBusIndex]))/1000.0;
				TempTime = ((float)(SensorEvent.timestamp)/1000.0) - FlushTime;
				if (TempTime > 0.0)
				{
					*BMEPresLastReadings[I2CBusIndex] = TempReading;
					*BMEPresLastReadingTimes[I2CBusIndex] = TempTime;
					BMEPressures[I2CBusIndex]->Push((double)TempTime,(double)TempReading);
					if (BMEPressures[I2CBusIndex]->NumDataValues() >= SampleCountMax)
					{
						TimeSpan  = BMEPressures[I2CBusIndex]->MaxX() - BMEPressures[I2CBusIndex]->MinX();
						Serial.print("BMEPres,");
						Serial.print(I2CBusIndex);
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->MeanY());
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->StandardDeviationY());
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->MinY());
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->MaxY());
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->Slope());
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->Intercept());
						Serial.print(",");
						Serial.print(BMEPressures[I2CBusIndex]->Correlation());
						Serial.print(",");
						Serial.print(TimeSpan);
						Serial.print("\n");
						BMEPressures[I2CBusIndex]->Clear();
						*BMEPresLastFlushTimes[I2CBusIndex] = millis();
					}
				}
				//else
				//{
				//	Serial.print("BME Pres time error ");
				//	Serial.print(TempTime);
				//	Serial.print("\n");
				//	BMEPressures[I2CBusIndex]->Clear();
				//	BMEPresFlushTime = millis();
				//}
			}
		}
	}
	SetNeoPixel(0,0,0);
}

void UpdateBMEHumidity()
{
	bool Status;
	sensors_event_t SensorEvent;
	float TempReading;
	float TempTime;
	double TimeSpan;
	float FlushTime;
	uint8_t ActiveChannel;
	SetNeoPixel(0,0,128);
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<BMECount; I2CBusIndex++)
	{
		ActiveChannel = BMEBus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		Status = BMEHumiSensors[I2CBusIndex]->getEvent(&SensorEvent);
		if (Status)
		{
			TempReading = SensorEvent.relative_humidity;
			if (TempReading != *BMEHumiLastReadings[I2CBusIndex])
			{
				FlushTime = (float)((*BMEHumiLastFlushTimes[I2CBusIndex]))/1000.0;
				TempTime = ((float)(SensorEvent.timestamp)/1000.0) - FlushTime;
				if (TempTime > 0.0)
				{
					*BMEHumiLastReadings[I2CBusIndex] = TempReading;
					*BMEHumiLastReadingTimes[I2CBusIndex] = TempTime;
					BMEHumidities[I2CBusIndex]->Push((double)TempTime,(double)TempReading);
					if (BMEHumidities[I2CBusIndex]->NumDataValues() >= SampleCountMax)
					{
						TimeSpan  = BMEHumidities[I2CBusIndex]->MaxX() - BMEHumidities[I2CBusIndex]->MinX();
						Serial.print("BMEHumi,");
						Serial.print(I2CBusIndex);
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->MeanY());
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->StandardDeviationY());
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->MinY());
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->MaxY());
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->Slope());
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->Intercept());
						Serial.print(",");
						Serial.print(BMEHumidities[I2CBusIndex]->Correlation());
						Serial.print(",");
						Serial.print(TimeSpan);
						Serial.print("\n");
						BMEHumidities[I2CBusIndex]->Clear();
						*BMEHumiLastFlushTimes[I2CBusIndex] = millis();
					}
				}
				//else
				//{
				//	Serial.print("BME Humi time error ");
				//	Serial.print(TempTime);
				//	Serial.print("\n");
				//	BMEHumidities[I2CBusIndex]->Clear();
				//	BMEHumiFlushTime = millis();
				//}
			}
		}
	}
	SetNeoPixel(0,0,0);
}

void UpdateTMPTemperature()
{
	bool Status;
	sensors_event_t SensorEvent;
	float TempReading;
	float TempTime;
	double TimeSpan;
	float FlushTime;
	uint8_t ActiveChannel;
	SetNeoPixel(128,0,128);
	for(uint8_t I2CBusIndex = 0; I2CBusIndex<TMPCount; I2CBusIndex++)
	{
		ActiveChannel = TMPBus[I2CBusIndex];
		I2CMultiplexer.selectChannel(ActiveChannel);
		Status = TMPSet[I2CBusIndex]->getEvent(&SensorEvent);
		if (Status)
		{
			TempReading = SensorEvent.temperature;
			bool ReadingDifferent = TempReading != *TMPLastReadings[I2CBusIndex];
			if (ReadingDifferent)
			{
				FlushTime = (float)( (*TMPLastFlushTimes[I2CBusIndex]) )/1000.0;
				TempTime = ((float)(SensorEvent.timestamp)/1000.0) - FlushTime;
				if (TempTime > 0.0)
				{
					*TMPLastReadings[I2CBusIndex] = TempReading;
					*TMPLastReadingTimes[I2CBusIndex] = TempTime;
					TMPTemperatures[I2CBusIndex]->Push((double)TempTime,(double)TempReading);
					if (TMPTemperatures[I2CBusIndex]->NumDataValues() >= SampleCountMax)
					{
						TimeSpan  = TMPTemperatures[I2CBusIndex]->MaxX() - TMPTemperatures[I2CBusIndex]->MinX();
						Serial.print("TMPTemp,");
						Serial.print(I2CBusIndex);
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->MeanY());
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->StandardDeviationY());
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->MinY());
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->MaxY());
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->Slope());
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->Intercept());
						Serial.print(",");
						Serial.print(TMPTemperatures[I2CBusIndex]->Correlation());
						Serial.print(",");
						Serial.print(TimeSpan);
						Serial.print("\n");
						TMPTemperatures[I2CBusIndex]->Clear();
						*TMPLastFlushTimes[I2CBusIndex] = millis();
					}
				}
				//else
				//{
				//	Serial.print("TMP Temp time error: ");
				//	Serial.print(TempTime);
				//	Serial.print("\n");
				//	TMPTemperatures[I2CBusIndex]->Clear();
				//	TMPTempFlushTime = millis();
				//}
			}
		}
	}
	SetNeoPixel(0,0,0);
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
		UpdateBMETemperature();
		UpdateBMEPressure();
		UpdateBMEHumidity();
		UpdateTMPTemperature();
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