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
#define Faixa_Min 50
#define Faixa_Max 60
#define TAM_FAIXA Faixa_Max-Faixa_Min
//#define FaixaG1 5
//#define FaixaG2 45
//#define FaixaLarg 10
#define Faixa_Min2 5
#define Faixa_Max2 15

bool pixel(int i, int j) {  // função que verifica o valor do pixel
    return (fb->buf[i * 96 + j]>150)?true:false;
}
void loop() {
  // Do nothing. Everything is done in another task by the web server
  fb = esp_camera_fb_get();
  //float F1=mediaLinha(Faixa_Min, Faixa_Max);
  //float F2=mediaLinha(Faixa_Min2, Faixa_Max2);
  //ajusteMotor((F1+F2)/2);
  //float erro=(mediaLinha(50,60)+fatorErroAngulo()*48)/2;
  float erroL=mediaLinha(50,60);
  float erro=erroL*(abs(erroL)/48) + 85*fatorErroAngulo();
  Serial.println(erro);
  ajusteMotor(erro);

  
  /*for(int i=40; i<60; i++){
    Serial.println();
    for(int j=0; j<fb->width; j++){
      Serial.print(pixel(i,j));
    }
  }
  Serial.println();
  Serial.println(fatorErroAngulo());
  Serial.println();

  delay(2000);*/
  esp_camera_fb_return(fb);


}

int confereExtremidades(){
  

}

void ajusteMotor(float erro){

  if(erro>1){
    analogWrite(IN2, SetPoint);
    analogWrite(IN3, SetPoint + erro);
  } /*else if (erro>30){
    analogWrite(IN2, SetPoint/2);
    analogWrite(IN3, SetPoint);
  }*/ else if (erro<-1){
    analogWrite(IN2, SetPoint - erro);
    analogWrite(IN3, SetPoint);
  } /*else if (erro<-30){
    analogWrite(IN2, SetPoint);
    analogWrite(IN3, SetPoint/2);
  }*/ else{
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

    for(int i = i_min+1, z = 0; i < i_max+1; i++, z++){
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
  
  erroFx2 = mediaLinha(40,50); // Variação dy || Se negativo, vira a esquerda, se positivo, a direita
  erroFx1 = mediaLinha(50,60);
  dy=erroFx1-erroFx2;
  hip = sqrt(pow(dy, 2) + pow(TAM_FAIXA, 2));

  sen = dy/hip; // Se erro = 0, seno do angulo = 0, portanto está seguindo uma linha

  return -sen; // Range da função vai pra 2
}

bool IDlinha(){
  int i=0;    //linha
  int j=0;    //coluna
  bool ashow=false;
  int count=0;

  for(int i = 0; i < 96; i++){ // Verifica coluna 1
    if (pixel(i,0)){ 
      count++;
    }else
      count=0;
    if (count >= 4){  // Diminui probabilidade de pequenos 1's serem considerados linha
      return 1; // Linha a esquerda
    }
  }
  count = 0;
  for(int i = 0; i < 96; i++){ // Verifica coluna 1
    
    if (pixel(i,95)){ 
      count++;
    }else
      count=0;
    if (count >= 4){  // Diminui probabilidade de pequenos 1's serem considerados linha
      return 2; // Linha a direita
    }
  }
  while((i+1)<fb->width){     
    ashow=false;   //idica se achou um 1 na linha de baixo
    while(j<fb->width){
      
      if(pixel(i,j)){
        if(pixel(i+1,j)){   //si tive um 1 logo embaixo do outro
          ashow=true;
          break;
        }
        else{
          int p=1;
          while((j-p)>=0 && (j+p)<fb->width){
              if(pixel(i,j+p-1))      //confere se tem um 1 na casa diagonal pra direita
                if(pixel(i+1,j+p)){
                  j=j+p;
                  ashow=true;
                  break;
                }

              if(pixel(i,j-p+1))      //confere se tem um 1 na casa diagonal da esquerda
                if(pixel(i+1,j-p)){
                  j=j-p;
                  ashow=true;
                  break;
                }
            p++;      //se n achar nenhum um confere expande para os lados
          }
        }
        break;
      }
    }
    if(ashow==false){       //se depois de tudo nao achar nenhum 1 antes de chegar na ultima linha da matriz não existe linha na matriz
      break;
    }
    i++;
  }
  return ashow;     //retorna true se achar linha e false se n
}
void IDquadrado(){

}

void IDfaixa(){

}






