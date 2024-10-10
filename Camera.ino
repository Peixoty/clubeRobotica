#include "esp_camera.h"
#include <math.h>
//#include <WiFi.h>
#include "BluetoothSerial.h"

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
#include "camera_pins.h"

//BluetoothSerial SerialBT;

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

#define IN1 4
#define IN2 2
#define IN3 12
#define IN4 13

#define SetPoint 45 // vel base do motor

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
  /*
  if(!SerialBT.begin("ESP32CAM")) {
    Serial.println("Erro ao iniciar o Bluetooth");
  } else {
    Serial.println("Bluetooth iniciado com sucesso. Agora você pode parear com o ESP32CAM.");
  }
  */
  //inicialização motor
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  analogWrite(IN4, 0);
  analogWrite(IN3, 0);

}

camera_fb_t *fb = NULL;
#define disLeituras 20 // distanciamento entre leituras
#define alturaLeitura 45 // maximo de 48
#define Carlos 150 // definição doq é preto


#define Roboi (95+9.5/0.15) // distancia da borda da matriz ate o centro do eixo
#define Roboj 47.5 // centro do eixo em coordenadas
#define difRodas (6/0.15) // distancia das rodas em pixels

#define difMotor 0.73 // balanceamento de motor

void printCamera(){ // printa camera em uma matriz
  for(int i=0; i<fb->width; i++){
    Serial.println();
    for(int j=0; j<fb->width; j++){
      Serial.print(pixel(i,j));
    }
  }
  Serial.println();
}


bool pixel(int i, int j) {  // função que verifica o valor do pixel
    return (fb->buf[i * 96 + j]>Carlos)?true:false; // converte o valor de 0 a 255 em booleano sendo os pontos brancos=1
}

int ji;

class Desafios{
  int quad=0, qlido=0;
  bool cruz=false;

  public:
  void checarDesafio(){ 
    if(quad>=0){
      countQuadrado(true); // true lado da direita // false lado da esquerda
    }
    if(quad<=0){
      cruz=false;
      countQuadrado(false); // se for direita quad é positivo se for esquerda quad é negativo
    }

    if(cruz){
      esp_camera_fb_return(fb);
      if(abs(quad)>1){
        //SerialBT.println("90");
        jmyself(quad/abs(quad));
      }
      fb = esp_camera_fb_get();
      qlido=0;
      quad=0;
      cruz=false;
    }
  }
  private:
  void countQuadrado(int sentido){ // true lado da direita // false lado da esquerda
    for(int i=fb->width-1; i>=0; i--){
      int count=0;
      while(pixel(i,(fb->width-1)*sentido) || pixel(i,(fb->width-3)*sentido+1)){
        count++;
        if(i<=2 || i>=fb->width-3){
          count=0;
          if(i>=fb->width-3){
            while(pixel(i,(fb->width-1)*sentido) || pixel(i,(fb->width-3)*sentido+1)){
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
          quad+=2*sentido-1;
          qlido=2*sentido-1;
        }
        else if(count>7){
          cruz=true;
        }
      }
    }
  }

  void jmyself(int sentido){ // negativo direita // positivo esquerda
    float LinhajA;
    bool direcaoGiro = (sentido == 1)? true:false;
    // Anda por 100ms confere se a linha está alinhada. Repete até ver uma linha com media na coluna 48
    analogWrite(IN2, 50*difMotor);
    analogWrite(IN3, 50);
    delay(200);
    analogWrite(IN2, 50*direcaoGiro*difMotor);
    analogWrite(IN3, 50*(!direcaoGiro));
    delay(500);

    do{  
      analogWrite(IN2, 45*direcaoGiro*difMotor);
      analogWrite(IN3, 45*(!direcaoGiro));
      fb = esp_camera_fb_get();
      ji=0;
      LinhajA=mediaLinha(20, false, ji);
      while(ji<fb->width-1){
        float LinhajN=mediaLinha(20, false, ji);
        if(abs(LinhajN-47.5)<abs(LinhajA-47.5)){
          LinhajA=LinhajN;
        }
      }
      //SerialBT.println(LinhajA);
      esp_camera_fb_return(fb);

    } while((LinhajA > (56 + direcaoGiro*5)) || (LinhajA < (40 - !direcaoGiro*5))); // Enquanto a linha não estiver num range de 16 do centro, continua virando

    analogWrite(IN2, 0);
    analogWrite(IN3, 0);

    delay(500);

    parteMotor(45);

  }
};

Desafios des;
void loop() {
  fb = esp_camera_fb_get(); // preenche o vetor com a leitura atual da camera
  //des.checarDesafio();
  //ajusteMotor(erroFx1, fatorErroAngulo(erroFx1, erroFx2)); // ajusta o motor com base no erro de centralização e angulação da linha
  seguirLinha(alturaLeitura);
  //analogWrite(IN2, SetPoint*difMotor);
  //analogWrite(IN3, SetPoint);
  //printCamera();
  esp_camera_fb_return(fb); // esvazia o vetor preenchido pela leitura da camera*/
}

void seguirLinha(int alt){
  ji=0;
  float LinhajA=mediaLinha(alt, false, ji);
  while(ji<fb->width-1){
    float LinhajN=mediaLinha(alt, false, ji);
    if(abs(LinhajN-47.5)<abs(LinhajA-47.5)){
      LinhajA=LinhajN;
    }
  }
  //Serial.println();
  //Serial.println(LinhajA);

  float Linhai=abs(47.5-LinhajA)+alt; // transformação da media das colunas na media de linhas
  float r=raio(Linhai, LinhajA); // manda as coordenadas lidas para o raio
  if(LinhajA!=0){
    ajusteMotor(r);
  }
}

void ajusteMotor(float r){
    analogWrite(IN2, SetPoint*abs((r+difRodas)/r)*difMotor);
    analogWrite(IN3, SetPoint*abs((r-difRodas)/r));
}

float mediaLinha(int a, bool corLinha, int inicio){// ponto medio da linha
  int som=0; // somatorio de colunas
  int peso=0; // numero de uns
  bool p=false;
  for(int j=inicio; j<fb->width; j++){
    int i=abs(47.5-j)+a;  //j e i fazem uma leitura em v da matriz
    bool x=(corLinha!=pixel(i,j));
    som+=x*j;
    peso+=x;
    //Serial.print(x);
    if(x){
      p=true;
    }else if(p){
      ji=j;
      return som/peso; // media das colunas lidas
    }
  }
  ji=fb->width;
  return 0; // media das colunas lidas
}
float raio(float i, float j){
  float di, dj, hip, cos, r;
  di=Roboi-i; // distancia entre as coordenadas do eixo do robo e da media da linha
  dj=Roboj-j; //

  hip=sqrt(di*di+dj*dj); // hipotenusa das distancias
  cos=dj/hip; // cos para descobri o raio

  
  return (hip/2)/cos; // raio da circunferencia
}

void parteMotor(int sp){
  int i;
  for(i = 1; i <= sp; i++){
    analogWrite(IN2, i);
    analogWrite(IN3, i);
    delay(50);
  }
}

