#include "esp_camera.h"
#include <math.h>
#include <WiFi.h>
#include "BluetoothSerial.h"

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "LAIC01"; 
const char *password = "NETLAIC01";

//Configuração dos pinos da câmera
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define IN1 2
#define IN2 4
#define IN3 12
#define IN4 13
#define SetPoint 50

//void startCameraServer();



void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 30000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_96X96;
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }


  /*WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("Conectado: ");
  Serial.print("IP: http://");
  Serial.print(WiFi.localIP());

  startCameraServer();*/


  //inicialização motor
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  analogWrite(IN2, SetPoint);

  digitalWrite(IN4, LOW);
  analogWrite(IN3, SetPoint);
}

camera_fb_t *fb = NULL;
#define Faixa_Min 43
#define Faixa_Max 53
#define TAM_FAIXA Faixa_Max-Faixa_Min
//#define FaixaG1 5
//#define FaixaG2 45
//#define FaixaLarg 10
#define Faixa_Min2 5
#define Faixa_Max2 15

bool pixel(int i, int j) {  // função que verifica o valor do pixel
    return (fb->buf[i * 96 + j]>144)?true:false;
}

void printCamera(){
  for(int i=40; i<60; i++){
    Serial.println();
    for(int j=0; j<fb->width; j++){
      Serial.print(pixel(i,j));
    }
  }
  //delay(2000);
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  fb = esp_camera_fb_get();

  ajusteMotor(mediaLinha(Faixa_Min,Faixa_Max), fatorErroAngulo());

  esp_camera_fb_return(fb);

}

void ajusteMotor(float erroLinha, float erroAngulo){

  if (erroAngulo > 0.2){
    analogWrite(IN2, SetPoint - 15);
    analogWrite(IN3, SetPoint + 75*erroAngulo);
  } else if (erroAngulo < -0.2){ 
    analogWrite(IN2, SetPoint - 75*erroAngulo);
    analogWrite(IN3, SetPoint - 15);
  }else if(erroLinha>0){
    analogWrite(IN2, SetPoint);
    analogWrite(IN3, SetPoint + (erroLinha*(abs(erroLinha)/48)));
  } else if (erroLinha<0){
    analogWrite(IN2, SetPoint - (erroLinha*(abs(erroLinha)/48)));
    analogWrite(IN3, SetPoint);
  } else{
    analogWrite(IN2, SetPoint);
    analogWrite(IN3, SetPoint);
  }
  
}

float calculaMedia(float *array, int tamArray){
  float soma = 0.0;

  for(int i = 0; i < tamArray; i++){
    soma += array[i];
  }
  return soma/tamArray;
}

float mediaLinha(int i_min, int i_max) { // Acha o centro da linha branca
    float soma = 0;
    float peso = 0;
    float erros[TAM_FAIXA];

    for(int i = i_min, z = 0; i < i_max; i++, z++){
      for (int j = 0; j < fb->width; j++) {
        int val = pixel(i, j);
        soma += val * j;
        peso += val;
      }
      erros[z] = (peso == 0)? 0.0 : (soma/peso)-48; // Vetor de erros de N faixas
    }

    return calculaMedia(erros,TAM_FAIXA);
}

float fatorErroAngulo(){
  float erroFx2, hip, sen, erroFx1, dy;
  
  erroFx2 = mediaLinha(33,43); // Variação dy || Se negativo, vira a esquerda, se positivo, a direita
  erroFx1 = mediaLinha(43,53);
  dy=erroFx1-erroFx2;
  hip = sqrt(pow(dy, 2) + pow(TAM_FAIXA, 2));

  sen = dy/hip; // Se erro = 0, seno do angulo = 0, portanto está seguindo uma linha

  return -sen; // Range da função vai pra 2
}

bool IDlinha(){
  


}
void IDquadrado(){

}

void IDfaixa(){

}






