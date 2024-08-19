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
#define MCP4661_ADDRESS 0x28  //Potenciometro

// Objetos y variables globales
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;
Adafruit_MCP4725 dac;

// Variables para almacenar valores del ADC
int16_t adc0, adc1;
int caso=0;
// Direcciones de los dispositivos I2C
#define ADS1115_ADDRESS 0x48
#define MCP4661_ADDRESS 0x28
#define MCP4725_ADDRESS 0x60
#define DAC_RESOLUTION 4095.0
#define ADC_RESOLUTION 26666.0  // Valor Practico diferente a valor real que es 2^16

//--------------------------------------------------------------------------------------------------------------------------------
//    Constantes de control
//--------------------------------------------------------------------------------------------------------------------------------
bool estado=true; //0 1_Vacio 2_Carga
float ki1, ki2, ki3, kv1, kv2, kv3;
float H_i, ei, ei_m1, ei_m2, ei_m3, ui , ui_m1, H_v, ev , ev_m1 , ev_m2 , ev_m3, uv, uv_m1;
double v_act = 0; 
double i_act = 0;

double voltage0 = 0.0;  // Convierte el valor del ADC0 a voltaje
double voltage1 = 0.0;  // Convierte el valor del ADC1 a voltaje

float valores[6][12] = {
        //sin carga                                     con carga
        {0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001,     0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001}, //5 ajustar para sin carga
        {0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001,     0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001}, //10
        {0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001,     0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001}, //15
        {0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001,     0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001}, //20
        {0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001,      0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001}, //25 //Espectacular estas constantes
        {0.5, 0.45, 0.01, 0.01, -0.0095, 0.00001,     0.1, 0.01, 0.001, 0.011, -0.0102, 0.00001}  //>25
};
float i_min_range[6]={
  0.06,   0.06,   0.06,   0.06,   0.06,   0.06,
  };
float i_min;
float sobrepaso[6]={
  1.2, 1.2,  1.3,  1.14, 1.15, 1.15
  };

float v_ref = 25;  //Tensión de referencia
float i_max = 1.5;
bool flag;

//pines encoder 1 --------------------------------------------------------------------------------------------------

const int encoderCLK = A1;  // Pin D12
const int encoderDT = A2;   // Pin D13
const int encoderSW = A3;   // Pin A0
int counter = 0;
int currentStateCLK;
int lastStateCLK;
bool buttonPressed = false;
// -------------------------------------------------------------------------------------------------------------------
const int pinCarga = 2;  // Pin donde que trabaja sobre el relé
// -------------------------------------------------------------------------------------------------------------------
unsigned long lastResetTime = 0; // Variable para almacenar el tiempo de la última ejecución de reset_variables
const unsigned long resetInterval = 5000; // Intervalo de 5 segundos en milisegundos
unsigned long lastExecutionTime = 0; // Variable para almacenar el tiempo de la última ejecución
const unsigned long executionInterval = 500; // Intervalo de 5 segundos en milisegundos
// Definicion de funciones--------------------------------------------------------------------------------------------
unsigned long startTime = 0;  // Tiempo de inicio para la medición
bool isBelowThreshold = false;  // Estado si v_act está por debajo del umbral
// -------------------------------------------------------------------------------------------------------------------
void reset_variables();
void constantes_control();
void lazo_control(float v_act, float i_act);
void control_sin_carga(float v_act, float i_act);
void actualizarDisplay(int16_t adc0, float voltage0, int16_t adc1, float voltage1, int dacValue, float volt_ref);
void setPotentiometer(byte channel, byte value);
void algoritmo_control();
void encoder_1();
void conexion_desconexion_carga();
void setup() {
  // Configuración de pines encoder
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderSW, INPUT_PULLUP);
  lastStateCLK = digitalRead(encoderCLK);     // Leer el estado inicial del CLK

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
  ads.setDataRate(RATE_ADS1115_860SPS);   // Configura la velocidad de muestreo a 250 SPS
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false); // true para modo continuo
  pinMode(2, OUTPUT); // Pin D2 Output Rele
  //digitalWrite(2, HIGH);  // Establece el pin D2 en estado alto (encendido)
  reset_variables();
  Serial.println("Inicialización");
  digitalWrite(2, HIGH);
  constantes_control();
  setPotentiometer(0,50);  // Canal 0, valor 128 (mitad del rango)
}

void loop() {
  encoder_1();
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  
  voltage0 = adc0 * (5.0 / ADC_RESOLUTION);  // Convierte el valor del ADC0 a voltaje
  voltage1 = adc1 * (5.0 / ADC_RESOLUTION);  // Convierte el valor del ADC1 a voltaje
  v_act = voltage0 * H_v;
  i_act = voltage1 * H_i;
  // Lazo de control Carga
  //Serial.println(i_act, 6);  // Imprime i_act con 6 decimales de precisión
  //Serial.println(v_act, 6);  // Imprime i_act con 6 decimales de precisión
  conexion_desconexion_carga(); //Anda
  constantes_control();
  algoritmo_control();
  float aux = (ui * DAC_RESOLUTION) / 5.0;  // Ajusta el voltaje a la resolución del DAC
  int dacValue = aux;
  dac.setVoltage(dacValue, false);  // Enviar valor al DAC
}

void conexion_desconexion_carga(){
   if (v_act < v_ref*1.05) {
    if (!isBelowThreshold) {
      startTime = millis();  // Si es la primera vez que está por debajo, guarda el tiempo actual
      isBelowThreshold = true;
    } else {
      if (millis() - startTime >= 2000) { // Han pasado 2 segundos con v_act por debajo del valor de referencia
        digitalWrite(pinCarga, HIGH);
      }
    }
  } else {    // Si v_act está por encima del valor de referencia
    isBelowThreshold = false;  // Reinicia el estado
    digitalWrite(pinCarga, LOW);
  }
}

void algoritmo_control(){
  if (millis() - lastExecutionTime >= executionInterval) {
    if (i_act > i_min && flag==true) { // || i_act >= 0.05
      estado = false; //Lazo con carga
      flag = false;
      reset_variables();
      lastExecutionTime = millis(); // Actualizar el tiempo de la última ejecución
    }
    if (v_act >= v_ref * 1.2 && i_act <= i_min) { 
      estado = true; //Lazo sin carga
      flag = true;
      //reset_variables();
      lastExecutionTime = millis(); // Actualizar el tiempo de la última ejecución
    }
    if(v_act <= v_ref * 0.2 && flag==true){ 
      estado = false; //Lazo con carga
      flag = false;
      reset_variables();
      lastExecutionTime = millis(); // Actualizar el tiempo de la última ejecución
    }
  }
  if (estado){
    control_sin_carga(v_act, i_act);
    //Serial.println("Lazo sin carga");
  } else {
    lazo_control(v_act, i_act);
    //Serial.println("Lazo con carga");
  }
}

void reset_variables() {
  Serial.println("Reseeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet");
  H_i = 0.6;
  ei = 0; ei_m1 = 0; ei_m2 = 0; ei_m3 = 0;
  ui = 0; ui_m1 = 0;
  H_v = 35 / 5;
  ev = 0; ev_m1 = 0; ev_m2 = 0; ev_m3 = 0;
  uv = 0; uv_m1 = 0;
}

void control_sin_carga(float v_act, float i_act) {
  //Lazo de tensión:
  ev = v_ref - v_act;
  uv = kv1 * ev + kv2 * ev_m1 + kv3 * ev_m2 + uv_m1;
  if (uv < 0.3) {
    uv = 0.3 - kv1 * ev - kv2 * ev_m1 - kv3 * ev_m2;
  }
  if (uv > 1.5) {
    uv = 1.5 - kv1 * ev - kv2 * ev_m1 - kv3 * ev_m2;
  }
  ev_m3 = ev_m2;
  ev_m2 = ev_m1;
  ev_m1 = ev;
  uv_m1 = uv;
  ei=0, ei_m1=0, ei_m2=0, ei_m3=0;
  ui=uv;
  ui_m1=ui;

  //Lazo de corriente:
  /*ei = uv - i_act;
  ui = ki1 * ei + ki2 * ei_m1 + ki3 * ei_m2 + ui_m1;
  if (ui > 1.5) {
    ui = 1.5 - ki1 * ei - ki2 * ei_m1 - ki3 * ei_m2;
  }
  if (ui < 0.3) {
    ui = 0.3 - ki1 * ei - ki2 * ei_m1 - ki3 * ei_m2;
  }
  ei_m3 = ei_m2;
  ei_m2 = ei_m1;
  ei_m1 = ei;
  ui_m1 = ui;*/
}

void lazo_control(float v_act, float i_act) {
  //Lazo de tensión:
  ev = v_ref - v_act;
  uv = kv1 * ev + kv2 * ev_m1 + kv3 * ev_m2 + uv_m1;
  if (uv < 0) {
    uv = 0 - kv1 * ev - kv2 * ev_m1 - kv3 * ev_m2;
  }
  if (uv > i_max) {
    uv = i_max - kv1 * ev - kv2 * ev_m1 - kv3 * ev_m2;
  }
  ev_m3 = ev_m2;
  ev_m2 = ev_m1;
  ev_m1 = ev;
  uv_m1 = uv;
  //Lazo de corriente:
  ei = uv - i_act;
  ui = ki1 * ei + ki2 * ei_m1 + ki3 * ei_m2 + ui_m1;
  if (ui > 1.5) {
    ui = 1.5 - ki1 * ei - ki2 * ei_m1 - ki3 * ei_m2;
  }
  if (ui < 0.3) {
    ui = 0.3 - ki1 * ei - ki2 * ei_m1 - ki3 * ei_m2;
  }
  ei_m3 = ei_m2;
  ei_m2 = ei_m1;
  ei_m1 = ei;
  ui_m1 = ui;
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

void constantes_control(){
  if (v_ref <= 5) {
  caso=0;
  } else if (v_ref <= 10) {
  caso=1;
  } else if (v_ref <= 15) {
  caso=2;
  } else if (v_ref <= 20) {
  caso=3;
  } else if (v_ref <= 25) {
  caso=4;
  } else{
    caso=5;
  };
  i_min=i_min_range[caso];

  if (estado) {
  ki1 = valores[caso][0];
  ki2 = valores[caso][1];
  ki3 = valores[caso][2];
  kv1 = valores[caso][3];
  kv2 = valores[caso][4];
  kv3 = valores[caso][5];
  
  } else {
  ki1 = valores[caso][6];
  ki2 = valores[caso][7];
  ki3 = valores[caso][8];
  kv1 = valores[caso][9];
  kv2 = valores[caso][10];
  kv3 = valores[caso][11];
  }
}

void setPotentiometer(byte channel, byte value) {
  Wire.beginTransmission(MCP4661_ADDRESS);
  Wire.write((channel == 0) ? 0x00 : 0x10); // Selecciona el canal (0 o 1)
  Wire.write(value); // Configura el valor del potenciómetro
  Wire.endTransmission();
}

void encoder_1(){
  currentStateCLK = digitalRead(encoderCLK); // Leer el estado del CLK
  if (currentStateCLK != lastStateCLK && currentStateCLK == LOW) { // Si el estado ha cambiado
    if (digitalRead(encoderDT) != currentStateCLK) {     // Si DT es diferente a CLK significa que el encoder está girando en sentido horario
      v_ref=v_ref+0.5;
    } else {
     v_ref=v_ref-0.1;
    }
    Serial.print("Valor: ");
    Serial.println(v_ref);
  }
  lastStateCLK = currentStateCLK;
  
  if (digitalRead(encoderSW) == LOW) {   // Leer el estado del botón
    if (!buttonPressed) {
      buttonPressed = true;
      Serial.println("Boton Pulsado");
    }
  } else {
    if (buttonPressed) {
      buttonPressed = false;
      Serial.println("Boton Liberado");
    }
  }
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