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

#define IN1 4
#define IN2 2
#define IN3 12
#define IN4 13

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

#define difMotor 0.6 // balanceamento de motor

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

class Desafios{
  int quad=0, qlido=0;
  bool cruz=false;

  public:
  void checarDesafio(){
    if(qlido==1){
      for(int i=fb->width-1; i>=fb->width-3; i--){
        for(int j=fb->width-1; j>=fb->width-2; j--){
          if(pixel(i,j)){
            qlido=0;
          }
        }
      }
    }
    else if(qlido==-1){
      for(int i=fb->width-1; i>=fb->width-3; i--){
        for(int j=0; j<=1; j++){
          if(pixel(i,j)){
            qlido=0;
          }
        }
      }
    }
    else{ 
      if(quad>=0){
        countQuadrado(true); // true lado da direita // false lado da esquerda
      }
      if(quad<=0){
        cruz=false;
        countQuadrado(false); // se for direita quad é positivo se for esquerda quad é negativo
      }
    }

    if(cruz){ // Se leu cruz
      esp_camera_fb_return(fb);
      if(quad==0){ // E não leu quadrado
        SerialBT.println("cruz?");
        fb = esp_camera_fb_get();
        cruz = confirmCruz(true);
        esp_camera_fb_return(fb);
        if(cruz){ // E leu cruz, tenta identificar a ré
          SerialBT.println("re?");
          idRe();
        }
      }
      else if(abs(quad)==1){ // E leu um quadrado, faz 90 graus
        SerialBT.println("90");
        jmyself(quad/abs(quad));
      }
      else if(abs(quad)>1){ // E leu mais que um quadrado, faz a rotatória
        SerialBT.println("rot");
        rot();
      }
      fb = esp_camera_fb_get();
      qlido=0;
      quad=0;
      cruz=false;
    } else if (quad == 0 && identificaInversaoCores()){ // Começa o desafio da faixa de pedestre se não tiver nenhum quadrado e identificar a faixa de pedestre
      SerialBT.println("faixa");
      desafioFaixaPedestre();
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

   bool confirmCruz(int sentido){ // true lado da direita // false lado da esquerda
    for(int i=fb->width-1; i>=0; i--){
      int count=0;
      while((pixel(i,(fb->width-1)*sentido) || pixel(i,(fb->width-3)*sentido+1)) && i!=0){
        count++;
        i--;
      }
      if(count>7){
        return true;
      }
    }
    return false;
  }

  void jmyself(int sentido){ // negativo direita // positivo esquerda
      float Linhaj;
      bool direcaoGiro = (sentido > 0)? true:false;
      bool momentosIniciais = true; // Caso esteja nas primeira medições, ele não lê a parte de cima do cruzamento cruz como jmyself concluído
      int contagemMomentos = 0;      

      posicionaRoboCruz();  // Posiciona a câmera em cima da cruz

      do{
        contagemMomentos++;
        if(contagemMomentos > 10){ // Numero de ms para começar a valer o valor de linhaj. Calibrar isso!
          momentosIniciais = false;
        }

        analogWrite(IN2, 50*(!direcaoGiro)*difMotor);
        analogWrite(IN3, 50*direcaoGiro);
        delay(30); // Mantemos o controle de quanto tempo dura o loop, mantendo a espera o suficiente para ele ler a media da linha

        fb = esp_camera_fb_get();
        Linhaj=mediaLinha(20, false);
        esp_camera_fb_return(fb);

      } while(!momentosIniciais && ((Linhaj > (56 + direcaoGiro*5)) || (Linhaj < (40 - !direcaoGiro*5)))); // Enquanto a linha não estiver num range do centro e não está nos momentos iniciais, continua virando

      analogWrite(IN2, 0);
      analogWrite(IN3, 0);

      delay(400);

      parteMotor(70);
  }

  void rot(){
    bool s;
    if(quad>0){
      s=true;
    }
    else{
      s=false;
    }
    jmyself(quad/abs(quad));
    //fb = esp_camera_fb_get();
    //esp_camera_fb_return(fb);
    Serial.println(quad);
    while(abs(quad)>1){
      fb = esp_camera_fb_get();
      seguirLinha(alturaLeitura);
      cruz=confirmCruz(s);
      if(cruz){
        quad-=(quad/abs(quad));
      }
      
      delay(5000);
      printCamera();
      Serial.println(quad);
      esp_camera_fb_return(fb);
    }
    jmyself(quad);
  }

  void idRe(){
    while(cruz){
      fb = esp_camera_fb_get();
      seguirLinha(alturaLeitura);
      cruz=false;
      countQuadrado(true);
      if(quad==0){
        countQuadrado(false);
      }
      cruz = confirmCruz(true);
      if(!cruz){
        cruz = confirmCruz(false);
      }
      esp_camera_fb_return(fb);
      if(quad!=0){
        Serial.print("re");
        marchaRe();
        break;
      }
    }
  }

  void marchaRe(){
    int count=0;
    while(count<4){
      count=0;
      fb = esp_camera_fb_get();
      seguirLinha(alturaLeitura);
      for(int i=0; i<30; i++){
        count+=pixel(i, (47.5-47.5*quad));
      }
      Serial.print(count);
      esp_camera_fb_return(fb);
    }
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    delay(1000);
    analogWrite(IN1, SetPoint*difMotor);
    analogWrite(IN4, SetPoint);
    count=0;
    while(count<4){
      count=0;
      fb = esp_camera_fb_get();
      for(int i=60; i<fb->width; i++){
        count+=pixel(i, (47.5-47.5*quad));
      }
      Serial.println(count);
      esp_camera_fb_return(fb);
    }
    analogWrite(IN1, 0);
    analogWrite(IN4, 0);
    delay(1000);
    jmyself(-quad);
  }

  bool identificaInversaoCores() {  // Conta número de brancos na vertical dos dois lados
    int i;
    int sensor = 0, numSensoresAtivos = 0; // Réplica do sensor de cor usando a câmera
    int j[5] = {15,31,47,63,79};  // Colunas que serão verificados (como a matriz vai de 0 a 95, subtrai 1)
    int countBranco[5] = {0,0,0,0,0}; // Número de brancos em cada coluna
    bool sensoresAtivados[5];

    //Armazena os counts de branco 
    while(sensor < 5){
      for(i = 0; i < fb->width){
        if(pixel(i,j[sensor])){
          countBranco[sensor]++;
        }
      }
      sensoresAtivados[sensor] = (countBranco[sensor] > 12)? 1:0;
      numSensoresAtivos += sensoresAtivados[sensor]; // Conta número de "sensores" ativos
      sensor++;
    }

    // 1 Faixa  -> Só linha principal
    // 2 Faixas -> Linha principal e quadrado
    // 3 Faixas -> Laterais são brancas e garante que se a linha estiver entre 2 faixas e apareça um quadrado não leia como inversão de cores
    // 4 Faixas -> Idealmente o caso correto de inversão de cores
    // 5 Faixas -> Cruzamento (caso a largura dele for maior que 12)
    if(numSensoresAtivos > 2 && sensoresAtivados[2] == 0){
      return true;
    } else{
      return false;
    }

  }

  int identificaFaixaPedestre() {
    int z = 0, j = 0, count = 0;
    bool aux = true;

    while (aux) {

      if (pixel(40, j) && pixel(40, j + 1) && pixel(40, j + 2)) {  // Achou uma faixa se encontrar 3 pixels brancos consecutivos 00001111111111100000000001111111
        count++;
        for (z = j; z < (fb->width) - 2; z++) {                             // Busca por um pixel preto
          if (!pixel(40, z) && !pixel(40, z + 1) && !pixel(40, z + 2)) {  // Achou um entre faixa (preto)
            j = z;                                                        // Atualiza de onde o próximo J vai começar
            break;
          }
          if (z == (fb->width) - 3) {  // Se z chegar no limite, quebra o while
            aux = false;
          }
        }
      }

      j++;

      if(j == fb->width - 3){
        aux = false;
      }

      if (count > 1) {
        //SerialBT.println("Faixa de Pedestre");
        return 1;  // 1 representa a faixa de pedestre
      }
    }

    if (count == 1) {
      //SerialBT.println("Volta a seguir a linha");
      return 2;  // 2 representa a linha que ele deve voltar a seguir
    } else if (count == 0) {
      //SerialBT.println("Tudo preto");
      return 0;  // 0 representa que não há linha a ser seguida
    }
  }

  void desafioFaixaPedestre() {
    bool semLinha = false;  // Inicialmente há linha a ser seguida
    int j, count = 0, estadoLinha;
    float Linhaj, Linhai, r;

    esp_camera_fb_return(fb);

    while (!semLinha) {  // Enquanto tiver linha a ser seguida
      fb = esp_camera_fb_get();

      // Segue a linha
      seguirLinha(10);

      for (j = 0; j < fb->width; j++) {
        if (!pixel(30, j)) {  //Se a 30 linha tem muito pixel preto
          count++;
        }
      }

      if (count > 90) {  // Se tiver mais de 90 pixeis pretos, ele chegou no fim da linha branca
        semLinha = true;  // Não Há linha

      } else {
        count = 0;
      }

      esp_camera_fb_return(fb);
    }

    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    delay(5000);

    while (semLinha) {  // Enquanto ele não estiver enxergando a linha
      fb = esp_camera_fb_get();
      estadoLinha = identificaFaixaPedestre();

      if (estadoLinha == 0 || estadoLinha == 1) {  // Se tiver faixa de pedestre ou tudo preto, continua reto
        analogWrite(IN2, 120);
        analogWrite(IN3, 120);
        esp_camera_fb_return(fb);
      } else if (estadoLinha == 2) {  // Se achar a linha de volta, volta a seguir linha
        esp_camera_fb_return(fb);
        return;
      }
    }
  }

};

Desafios des;
void loop() {
  fb = esp_camera_fb_get(); // preenche o vetor com a leitura atual da camera
  des.checarDesafio();
  seguirLinha(alturaLeitura);
  esp_camera_fb_return(fb); // esvazia o vetor preenchido pela leitura da camera
}

void seguirLinha(int alt){
  float Linhaj=mediaLinha(alt, false);
  float Linhai=abs(47.5-Linhaj)+alt; // transformação da media das colunas na media de linhas
  float r=raio(Linhai, Linhaj); // manda as coordenadas lidas para o raio
  if(Linhaj!=0){
    ajusteMotor(r);
  }
}

void ajusteMotor(float r){
  analogWrite(IN2, SetPoint*abs(((r+difRodas)/r))*difMotor);
  analogWrite(IN3, SetPoint*abs(((r-difRodas)/r)));
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
  return (peso == 0)? 0:som/peso; // media das colunas lidas
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
  for(i = 0; i <= sp; i+=15){
    analogWrite(IN2, i*difMotor);
    analogWrite(IN3, i);
    delay(50);
  }
  delay(400);
}

void posicionaRoboCruz(){
  int linha = 40, coluna;
  int count = 0;

  while(count < 30){
    count = 0;
    for(coluna = 0; coluna < fb->width; coluna++){
      if(pixel(linha,coluna)){
        count++;  // Número de brancos na linha
      }
    }
  }
}
