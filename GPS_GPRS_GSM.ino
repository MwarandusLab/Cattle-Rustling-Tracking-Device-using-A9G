#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_SSD1306.h> // OLED display library

static const int RXPin = 3 , TXPin = 4;
String s = "www.google.com/maps/dir/";

unsigned long interval = 10000;
static const uint32_t GPSBaud = 9600;
unsigned long previousMillis = 0;
int data_counter;

const size_t BUFSIZE = 300;
char f_buffer[BUFSIZE];
float *f_buf = (float*)f_buffer;

TinyGPSPlus gps;// The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin);// The serial connection to the GPS device

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Initialize with the I2C address of your OLED display

bool gsmConnected = false;
bool gpsConnected = false;

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("GSM: Connecting");
  display.println("GPS: Connecting");
  display.display();
  
  Serial.println("Starting...");
  ss.println("\r");
  ss.println("AT\r");
  delay(10);

  ss.println("\r");
  ss.println("AT+GPS=1\r");

  delay(100);
  ss.println("AT+CREG=2\r");
  delay(5000);

  ss.print("AT+CREG?\r");
  ss.println("AT+CGATT=1\r");
  delay(5000);

  ss.println("AT+CGDCONT=1,\"IP\",\"WWW\"\r");
  delay(5000);

  ss.println("AT+LOCATION=1\r");
  ss.println("AT+CGACT=1,1\r");
  delay(5000);

  //Initialize ends
  //Initialize GPS
  ss.println("\r");
  ss.println("AT+GPS=1\r");
  delay(1000);

  ss.println("AT+GPSMD=1\r");   // Change to only GPS mode from GPS+BDS, set to 2 to revert to default.
  ss.println("AT+GPSRD=10\r");
  delay(100);

  // set SMS mode to text mode
  ss.println("AT+CMGF=1\r");
  delay(1000);

  ss.println("AT+LOCATION=2\r");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("GSM: Connecting");
  display.println("GPS: Connecting");
  display.display();

  Serial.println("Setup Executed");
  Serial.println("Sending Message");

  ss.println("AT+CMGF=1\r");
  delay(1000);

  ss.println("AT+CNMI=2,2,0,0,0\r");
  delay(1000);

  ss.print("AT+CMGS=\"+254748613509\"\r");//Replace this with your mobile number
  delay(1000);
  ss.print(s);
  ss.write(0x1A);
  delay(1000);
}

void loop() {
  if (Serial.available()) {
    ss.write(Serial.read());
  }
  if (ss.available()) {
    Serial.write(ss.read());
  }

  smartDelay(2000);

  // Check if GPS data is available and valid
  if (!gps.location.isValid()) {
    Serial.println(F("No valid GPS data received: check wiring or GPS signal"));
    return;
  }

  unsigned long currentMillis = millis();

  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    send_gps_data();
    previousMillis = currentMillis;
  }
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void send_gps_data(){
  if (gps.location.lat() == 0 || gps.location.lng() == 0)
  {
    Serial.println("Return Executed");
    return;
  }

  data_counter++;

  Serial.print("Latitude (deg): ");
  f_buf[data_counter] = gps.location.lat();
  Serial.println(f_buf[data_counter]);

  Serial.print("Longitude (deg): ");
  f_buf[data_counter + 1] = gps.location.lng();
  Serial.println(f_buf[data_counter + 1]);

  Serial.println(data_counter);
  Serial.println();

  s += String(gps.location.lat(), 6);
  s += ",";
  s += String(gps.location.lng(), 6);
  s += "/";

  Serial.println(s);

  if (data_counter >= 10)
  {
    data_counter = 0;

    Serial.println("Sending Message");

    ss.println("AT+CMGF=1\r");
    delay(1000);

    ss.println("AT+CNMI=2,2,0,0,0\r");
    delay(1000);

    ss.print("AT+CMGS=\"+254748613509\"\r");//Replace this with your mobile number
    delay(1000);
    ss.print(s);
    ss.write(0x1A);
    delay(1000);
    s = "www.google.com/maps/dir/";
  }
}