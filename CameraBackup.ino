#include "esp_camera.h"
#include <math.h>
//#include <WiFi.h>
#include "BluetoothSerial.h"

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
#include "camera_pins.h"

BluetoothSerial SerialBT;

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
#define IN2 15
#define IN3 12
#define IN4 13

#define SetPoint 60 // vel base do motor

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

  if(!SerialBT.begin("ESP32CAM")) {
    Serial.println("Erro ao iniciar o Bluetooth");
  } else {
    Serial.println("Bluetooth iniciado com sucesso. Agora você pode parear com o ESP32CAM.");
  }

  //inicialização motor
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  analogWrite(IN2, LOW);

  digitalWrite(IN4, LOW);
  analogWrite(IN3, LOW);

}

camera_fb_t *fb = NULL;
#define disLeituras 20 // distanciamento entre leituras
#define alturaLeitura 45 // maximo de 48
#define Carlos 150 // definição doq é preto


#define Roboi (95+9.5/0.1) // distancia da borda da matriz ate o centro do eixo
#define Roboj 47.5 // centro do eixo em coordenadas
#define difRodas (6/0.1) // distancia das rodas em pixels

#define difMotor 0.78 // balanceamento de motor

void printCamera(){ // printa camera em uma matriz
  for(int i=0; i<fb->width; i++){
    Serial.println();
    for(int j=0; j<fb->width; j++){
      Serial.print(pixel(i,j));
    }
  }
  Serial.println();
  delay(2000);
}
void printCameraBT(){
  for(int i=0; i<fb->width; i++){
    for(int j=0; j<fb->width; j++){
      if (SerialBT.available()) {
        SerialBT.print(pixel(i,j));
      }
    }
    if (SerialBT.available()) {
        SerialBT.println();
      }
  }
  if (SerialBT.available()) {
    SerialBT.println();
  }
  //delay(2000);
}


bool pixel(int i, int j) {  // função que verifica o valor do pixel
    return (fb->buf[i * 96 + j]>Carlos)?true:false; // converte o valor de 0 a 255 em booleano sendo os pontos brancos=1
}

class Desafios{
  int quad=0;
  bool cruz=false, qlidoE=false;

  public:
  void checarDesafio(){
    /*
    if (SerialBT.available()) {

      SerialBT.print("quad: ");
      SerialBT.println(quad);
      SerialBT.print("cruz: ");
      SerialBT.println(cruz);
    }*/
    if(qlidoE){
      for(int i=fb->width-1; i>=fb->width-3; i--){
        for(int j=fb->width-1; j>=fb->width-2; j--){
          if(pixel(i,j)){
            qlidoE=false;
          }
        }
      }
    }
    else{
      countQuadrado(false);
    }
    if(cruz){
      if(quad==0){
        
      }
      else if(abs(quad)==1){
        jmyself();
      }
      else if(abs(quad)>1){

      }
    }
  }
  private:
  void countQuadrado(bool sentido){
    for(int i=fb->width-1; i>=0; i--){
      int count=0;
      while(pixel(i,fb->width-1) || pixel(i,fb->width-2)){
        count++;
        if(i<=2 || i>=fb->width-3){
          count=0;
          if(i>=fb->width-3){
            while(pixel(i,fb->width-1) || pixel(i,fb->width-2)){
              i--;
            }
          }
          else if(i<=2){
            i=0;
          }
          break;
        }
        i--;
      }
      if(count!=0){
        if(count>25){
          quad++;
          qlidoE=true;
        }
        else if(count>7){
          cruz=true;
        }
      }
    }
  }

  void jmyself(){
    analogWrite(IN2, 25-((quad/abs(quad))*25));
    analogWrite(IN3, 25+((quad/abs(quad))*25));
    delay(1000);
    analogWrite(IN2, 60);
    analogWrite(IN3, 60);
    delay(200);
    quad=0;
    cruz=false;
  }
};

Desafios des;
void loop() {
  
  fb = esp_camera_fb_get(); // preenche o vetor com a leitura atual da camera

  //ajusteMotor(erroFx1, fatorErroAngulo(erroFx1, erroFx2)); // ajusta o motor com base no erro de centralização e angulação da linha
  float Linhaj=mediaLinha(alturaLeitura, false);
  float Linhai=abs(47.5-Linhaj)+alturaLeitura; // transformação da media das colunas na media de linhas
  float r=raio(Linhai, Linhaj); // manda as coordenadas lidas para o raio
  ajusteMotor(r);
  //printCameraBT();
  //printCamera();
  //analogWrite(IN2, SetPoint*0.78);
  //analogWrite(IN3, SetPoint);
  des.checarDesafio();
  esp_camera_fb_return(fb); // esvazia o vetor preenchido pela leitura da camera
}

void ajusteMotor(float r){
  analogWrite(IN2, SetPoint*abs((r+difRodas)/r)*difMotor);
  
  analogWrite(IN3, SetPoint*abs((r-difRodas)/r));
}
float mediaLinha(int a, bool corLinha){// ponto medio da linha
  int som=0; // somatorio de colunas
  int peso=0; // numero de uns
  for(int j=0; j<fb->width; j++){
    int i=abs(47.5-j)+a;  //j e i fazem uma leitura em v da matriz
    bool x=(corLinha!=pixel(i,j));
    som+=x*j;
    peso+=x;
  }
  return som/peso; // media das colunas lidas
}
float raio(float i, float j){
  float di, dj, hip, cos, r;
  di=Roboi-i; // distancia entre as coordenadas do eixo do robo e da media da linha
  dj=Roboj-j; //

  hip=sqrt(di*di+dj*dj); // hipotenusa das distancias
  cos=dj/hip; // cos para descobri o raio

  
  return (hip/2)/cos; // raio da circunferencia
}

/*
bool countQuadrado(){
  int count=0;   
  int ql=0;
  bool cruzl=false;
  bool cruz=false;
  int c=0;
  for(int i=fb->width-1; i>=0; i--){
    while(pixel(i,fb->width-1) || pixel(i,fb->width-2)){
      i--;
      count++;
      if (i=0){
        ql=0;
        cruzl=false;
      }
      else if (count > 26){
        ql=1;
        cruzl=false;
      }
      else if (count > 6){
        cruzl=true;
      }
    }
    if(cruzl){
    cruz=true;
    }
    
    Serial.println();
    Serial.println(count);
    count=0;
    q+=ql;
  }

  if(q==0){
    if(cruz){
      c++;
    }
    
    for(int i=fb->width-1; i>=0; i--){
      int c=0;
      while(pixel(i,0) || pixel(i,1)){
        i--;
        count++;
        if (i=0){
        ql=0;
        }
        if (count > 30){
          ql=-1;
          cruzl=false;
        }
        else if (count > 5){
          cruzl=true;
        }
      }
      if(q==0 && c==1 && cruzl){
        c++;
      }
      count=0;
      q+=ql;
    }
  }
  return q;
}
*/





