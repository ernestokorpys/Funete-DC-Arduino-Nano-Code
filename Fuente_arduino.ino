#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definiciones de pantalla OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define ADS1115_ADDRESS_ADDR_GND 0x48
#define MCP4661_ADDRESS 0x2F  //Potenciometro

// Objetos y variables globales
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;
Adafruit_MCP4725 dac;
unsigned long previousMillis = 0;
unsigned long startMillis;
unsigned long endMillis;
unsigned long conversionTime;
unsigned long previousDisplayMillis = 0;
const unsigned long displayInterval = 1000 * 4;  // 1 minuto en milisegundos
// Variables para almacenar valores del ADC
int16_t adc0, adc1;

// Direcciones de los dispositivos I2C
#define ADS1115_ADDRESS 0x48
#define MCP4725_ADDRESS 0x60
#define DAC_RESOLUTION 4095.0
#define ADC_RESOLUTION 26666.0  // Valor Practico diferente a valor real que es 2^16

//--------------------------------------------------------------------------------------------------------------------------------
//    Constantes de control
//--------------------------------------------------------------------------------------------------------------------------------
#define ki1 0.1
#define ki2 0.01
#define ki3 0.001
#define kv1 0.006
#define kv2 -0.005
#define kv3 0.0000001

//--------------------------------------------------------------------------------------------------------------------------------
//    Constantes de control
//--------------------------------------------------------------------------------------------------------------------------------
float H_i = 0.6;
float ei = 0, ei_m1 = 0, ei_m2 = 0, ei_m3 = 0;
float ui = 0, ui_m1 = 0;
float i_act = 0;
float H_v = 30 / 5;
float ev = 0, ev_m1 = 0, ev_m2 = 0, ev_m3 = 0;
float uv = 0, uv_m1 = 0;
float v_act = 0;
float v_ref = 10;  //Tensión de referencia
float i_max = 0.2;

void lazo_control(float v_act, float i_act);
void actualizarDisplay(int16_t adc0, float voltage0, int16_t adc1, float voltage1, int dacValue, float volt_ref);
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicialización de display OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  initDisplay();

  // Inicialización de ADC y DAC
  ads.begin(ADS1115_ADDRESS);
  dac.begin(MCP4725_ADDRESS);
  // Configura la velocidad de muestreo a 250 SPS
  ads.setDataRate(RATE_ADS1115_860SPS);
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false); // true para modo continuo

  // Pin D2 Output
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);  // Establece el pin D2 en estado alto (encendido)

  // Configura el potenciómetro digital MCP4661
  setPotentiometer(0x00, 10);  // Configura el canal 0 del potenciómetro al maximo de su valor (0-255)
  Serial.println("Inicialización");
}

void loop() {
  // Leer valores del ADC
  startMillis = millis();
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  // Convertir valores a voltaje (referencia de 5V) Valores sensados
  float voltage0 = adc0 * (5.0 / ADC_RESOLUTION);  // Convierte el valor del ADC0 a voltaje
  float voltage1 = adc1 * (5.0 / ADC_RESOLUTION);  // Convierte el valor del ADC1 a voltaje
  lazo_control(voltage0, voltage1);                //Se llama a la función que calcula la acción de control
  //Serial.println("Voltaje 0:", voltage0);
  //Serial.println("Voltaje 1:", voltage1);

  // Valor de acción de control
  float volt_ref = ui;                            // Voltaje de la acción de control
  float aux = (volt_ref * DAC_RESOLUTION) / 5.0;  // Ajusta el voltaje a la resolución del DAC
  int dacValue = aux;
  dac.setVoltage(dacValue, false);  // Enviar valor al DAC

  // Actualizar el display cada minuto
  unsigned long currentMillis = millis();
  if (currentMillis - previousDisplayMillis >= displayInterval) {
    actualizarDisplay(adc0, voltage0, adc1, voltage1, dacValue, volt_ref);
    previousDisplayMillis = currentMillis;
  }

  endMillis = millis();
  // Calcular y mostrar el tiempo de conversión
  conversionTime = endMillis - startMillis;

  //Serial.print("Tiempo de conversión ADC: ");
  //Serial.print(conversionTime);
  //Serial.println(" ms");
  // Mostrar valores en el monitor serial OLED
}

void initDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hola Ernesto");
  display.setCursor(0, 10);
  display.println("Bienvenido");
  display.display();
  delay(1000 * 2);
}

void setPotentiometer(byte pot, byte value) {
  Wire.beginTransmission(MCP4661_ADDRESS);
  Wire.write(pot);    // Selecciona el canal del potenciómetro
  Wire.write(value);  // Establece el valor del potenciómetro
  Wire.endTransmission();
}

void lazo_control(float v_act, float i_act) {
  v_act = v_act * H_v;
  i_act = i_act * H_i;
  //Lazo de tensión:
  ev = v_ref - v_act;
  uv = kv1 * ev_m1 + kv2 * ev_m2 + kv3 * ev_m3 + uv_m1;
  //if (ev < -2) {
  //  uv = uv - uv_m1;
  //}
  if (uv < 0) {
    uv = 0 - kv1 * ev_m1 - kv2 * ev_m2 - kv3 * ev_m3;
  }
  if (uv > i_max) {
    uv = i_max - kv1 * ev_m1 - kv2 * ev_m2 - kv3 * ev_m3;
  }
   Serial.println(kv1 * ev_m1);
  ev_m3 = ev_m2;
  ev_m2 = ev_m1;
  ev_m1 = ev;
  uv_m1 = uv;
  //Lazo de corriente:
  //uv=0.2;
  ei = uv - i_act;
  ui = ki1 * ei_m1 + ki2 * ei_m2 + ki3 * ei_m3 + ui_m1;
  if (ui > 1.5) {
    ui = 1.5 - ki1 * ei_m1 - ki2 * ei_m2 - ki3 * ei_m3;
  }
  if (ui < 0.3) {
    ui = 0.3 - ki1 * ei_m1 - ki2 * ei_m2 - ki3 * ei_m3;
  }
  ei_m3 = ei_m2;
  ei_m2 = ei_m1;
  ei_m1 = ei;
  ui_m1 = ui;
 
}

void actualizarDisplay(int16_t adc0, float voltage0, int16_t adc1, float voltage1, int dacValue, float volt_ref) {
  // Mostrar valores en el OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("ADC0: ");
  display.print(adc0);
  display.print("  V: ");
  display.println(voltage0, 3);
  display.setCursor(0, 10);
  display.print("ADC1: ");
  display.print(adc1);
  display.print("  V: ");
  display.println(voltage1, 3);
  display.setCursor(0, 20);
  display.print("DAC: ");
  display.print(dacValue);
  display.print("  V:");
  display.println(volt_ref, 3);
  display.display();
}