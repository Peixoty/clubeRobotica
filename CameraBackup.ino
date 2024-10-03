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
#define LED 4

#define SetPoint 50 // vel base do motor

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

  //LED câmera
  pinMode(LED, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN4, LOW);
  digitalWrite(IN3, LOW);

  digitalWrite(LED, LOW);
}

camera_fb_t *fb = NULL;

#define alturaLeitura 45 // maximo de 48
#define Carlos 150 // definição doq é preto


#define Roboi (95+9.5/0.1) // distancia da borda da matriz ate o centro do eixo
#define Roboj 47.5 // centro do eixo em coordenadas
#define difRodas (6/0.1) // distancia das rodas em pixels

#define difMotor 0.7 // balanceamento de motor

void printCamera(){ // printa camera em uma matriz
  for(int i=40; i<41; i++){
    Serial.println();
    for(int j=0; j<fb->width; j++){
      Serial.print(pixel(40,j));
    }
  }
  Serial.println();
  delay(2000);
}


bool pixel(int i, int j) {  // função que verifica o valor do pixel
    return (fb->buf[i * 96 + j]>Carlos)?true:false; // converte o valor de 0 a 255 em booleano sendo os pontos brancos=1
}

class Desafios{
  int q=0, quad=0;
  float qcord=0;
  bool cruz=false;

  public:
  void checarDesafio(){
    countQuadrado();
    
    if(cruz){
      quad=0;
      qcord=0;
    }
  }
  private:
  void countQuadrado(){
    for(int i=fb->width-1; i>=0; i--){
      int count=0;
      q=0;
      while(pixel(i,fb->width-1) || pixel(i,fb->width-2)){
        count++;
        q+=i;
        if(i<=2 || i>=fb->width-3){
          count=0;
          q=0;
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
      if(qcord>i+1 && qcord<=i+1+count){
        count=0;
      }
      if(count!=0){
        if(count>25){
          quad++;
          qcord=q/count;
          break;
        }
        else if(count>7){
          cruz=true;
        }
      }
    }
  }
};

Desafios des;

void loop() {

  analogWrite(LED, 0);
  
  fb = esp_camera_fb_get(); // preenche o vetor com a leitura atual da camera

  if (identificaInversaoCores()){ // Se a linha for preta, entra pro desafio
    desafioFaixaPedestre();

  } else{
    float Linhaj = mediaLinha(alturaLeitura, false);
    float Linhai = abs(47.5-Linhaj)+alturaLeitura; // transformação da media das colunas na media de linhas
    float r = raio(Linhai, Linhaj); // manda as coordenadas lidas para o raio
  
    ajusteMotor(r);
  }
  
  esp_camera_fb_return(fb); // esvazia o vetor preenchido pela leitura da camera
}

void ajusteMotor(float r){
  analogWrite(IN2, SetPoint*abs((r+difRodas)/r)*difMotor);
  
  analogWrite(IN3, SetPoint*abs((r-difRodas)/r));
}
// mediaLinha original
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

bool identificaInversaoCores(){
  int j;
  int countPreto = 0, countBranco = 0;

  // Faz a contagem dos pixels na linha 10
  for(j = 0; j < fb->width; j++){
    if(pixel(10,j)){
      countBranco++;
    } else {
      countPreto++;
    }
  }

  if(countPreto>=countBranco){
    return false; // A linha é branca
  } else if (countBranco > countPreto){
    return true; // Linha é preta
  }
}

int identificaFaixaPedestre(){ 
  int z, j, ultimaFaixaJ = 0, count = 0;

  for(j = ultimaFaixaJ; j < fb->width-2; j++){  // Busca por um pixel branco
    if(pixel(40,j) && pixel(40,j+1) && pixel(40,j+2)){ // Achou uma faixa
      count++;
      printCamera();
      for(z = j; z < fb->width-2; z++){ // Busca por um pixel preto
        if(!pixel(40,z) && !pixel(40,z+1) && !pixel(40,z+2)){  // Achou um entre faixa (preto)
          ultimaFaixaJ = z; // Atualiza de onde o próximo J vai começar
          Serial.println(ultimaFaixaJ);
          break;
        }
      }
    }
    
    if(count > 1){
      Serial.println("Faixa de Pedestre");
      return 1; // 1 representa a faixa de pedestre
    } 
  }

  if (count == 1){
      Serial.println("Volta a seguir a linha");
      return 2; // 2 representa a linha que ele deve voltar a seguir
  } else if (count == 0){
      Serial.println("Tudo preto");
      return 0; // 0 representa que não há linha a ser seguida
  }

}

void desafioFaixaPedestre(){
  bool semLinha = false;  // Inicialmente há linha a ser seguida
  int j, count = 0, estadoLinha;
  float Linhaj, Linhai, r;

  esp_camera_fb_return(fb);
  
  while(!semLinha){ // Enquanto tiver linha a ser seguida
    fb = esp_camera_fb_get();

    Serial.println("Reconheceu desafio");
    // Segue a linha
    Linhaj = mediaLinha(10, true); // Coloca o vértice do V na linha 10
    Linhai = abs(47.5-Linhaj)+10; // transformação da media das colunas na media de linhas
    r = raio(Linhai, Linhaj); // manda as coordenadas lidas para o raio

    ajusteMotor(r);

    for(j = 0; j < fb->width; j++){
      if(!pixel(20,j)){ //Se a 10 linha tem muito pixel preto
        count++;
      }
    }

    if (count > 90){  // Se tiver mais de 90 pixeis pretos, ele chegou no fim da linha branca
      Serial.println("Espera 5 segundos");
      semLinha = true; // Não Há linha

    } else{
      count = 0;
    }

    esp_camera_fb_return(fb);
  }

  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  delay(5000);

  while(semLinha){  // Enquanto ele não estiver enxergando a linha
    fb = esp_camera_fb_get();
    estadoLinha = identificaFaixaPedestre();

    if(estadoLinha == 0 || estadoLinha == 1){ // Se tiver faixa de pedestre ou tudo preto, continua reto
      analogWrite(IN2, SetPoint);
      analogWrite(IN3, SetPoint);
      esp_camera_fb_return(fb);
    } else if (estadoLinha == 2){ // Se achar a linha de volta, volta a seguir linha
      esp_camera_fb_return(fb);
      return;
    }

  }
}