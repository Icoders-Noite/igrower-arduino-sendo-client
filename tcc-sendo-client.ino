#include <ArduinoJson.h>
#include <WiFiEspClient.h>
#include "dht.h"
#include <Buzzer.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>
#include "WiFiEsp.h" //INCLUSÃO DA BIBLIOTECA
#include "SoftwareSerial.h"//INCLUSÃO DA BIBLIOTECA


//sensor dht11
#define pinoDHT11 A5 //PINO ANALÓGICO UTILIZADO PELO DHT11
//buzzer
#define  pinoBuzzer 5
// Pino ligado ao CS do modulo
#define chipSelect 4
//valvula
#define pin_valvula  8
//luz
#define pin_luz  9
// sensor solo
#define sensor_solo  A4

Buzzer buzzer(pinoBuzzer);

dht DHT; //VARIÁVEL DO TIPO DHT

//Variaveis para cartao SD
String parameter;
byte line;
String configs[2];

//parametros

String ssid = "", password = "";


int8_t luzStatus = 0;

int8_t  analogSoloSeco = 400; //VALOR MEDIDO COM O SOLO SECO (VOCÊ PODE FAZER TESTES E AJUSTAR ESTE VALOR)
int8_t  analogSoloMolhado = 150; //VALOR MEDIDO COM O SOLO MOLHADO (VOCÊ PODE FAZER TESTES E AJUSTAR ESTE VALOR)
int8_t percSoloSeco = 0; //MENOR PERCENTUAL DO SOLO SECO (0% - NÃO ALTERAR)
int8_t percSoloMolhado = 100;

unsigned long millisTarefa = millis();
char* getReq = "GET /Icoders-Noite/api-fake-test/arduino HTTP/1.1 ";
//wifi
SoftwareSerial Serial1(6, 7);
int status = WL_IDLE_STATUS; //STATUS TEMPORÁRIO ATRIBUÍDO QUANDO O WIFI É INICIALIZADO E PERMANECE ATIVO
//ATÉ QUE O NÚMERO DE TENTATIVAS EXPIRE (RESULTANDO EM WL_NO_SHIELD) OU QUE UMA CONEXÃO SEJA ESTABELECIDA
//(RESULTANDO EM WL_CONNECTED)

int solo =  -1;
int humidity = -1;
int temperature = -1;

void setup() {

  Serial.begin(9600); //INICIALIZA A SERIAL
  delay(2000); //INTERVALO DE 2 SEGUNDO ANTES DE INICIAR
  start();
  while (!Serial);
  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(chipSelect))
  {
    Serial.println(F("Card failed, or not present"));
    errorInicializacao(3, 200, 500);
  }
  Serial.println(F("card initialized."));
  File myFile = SD.open("CONFIG.txt");
  if (myFile)
  {
    while (myFile.available())
    {
      char c = myFile.read();
      if (isPrintable(c))
      {
        parameter.concat(c);
      }
      else if (c == '\n')
      {
        Serial.println(parameter);
        configs[line] = parameter;
        parameter = "";
        line++;
      }
    }
  }

  myFile.close();
  configParametros();
  pinMode(pin_valvula, OUTPUT);
  pinMode(pin_luz, OUTPUT);
  digitalWrite(pin_valvula, HIGH);
  digitalWrite(pin_luz, HIGH);
  pinMode(sensor_solo, INPUT);

  Serial1.begin(9600); //INICIALIZA A SERIAL PARA O ESP8266
  WiFi.init(&Serial1); //INICIALIZA A COMUNICAÇÃO SERIAL COM O ESP8266
  WiFi.config(IPAddress(192, 168, 15, 110)); //COLOQUE UMA FAIXA DE IP DISPONÍVEL DO SEU ROTEADOR
  //INÍCIO - VERIFICA SE O ESP8266 ESTÁ CONECTADO AO ARDUINO, CONECTA A REDE SEM FIO E INICIA O WEBSERVER
  if (WiFi.status() == WL_NO_SHIELD) {
    errorInicializacao(3, 200, 500);
  }
  configWifi();
  //quando está tudo certo
  musica();

}

void loop() {

  configWifi();
  // Connect to HTTP server
  WiFiEspClient client;
  char url[80] = "";
  if ((millis() - millisTarefa) > 300000) {
    millisTarefa = millis();
    solo =  sensorSolo();
    DHT.read11(pinoDHT11);
    delay(2000);
    humidity = DHT.humidity;
    temperature = DHT.temperature;
    delay(100);
  }
  //sprintf(url, "%s?dht=%i/%i&solo=%i&luz=%i HTTP/1.1", getReq, humidity, temperature, solo, luzStatus);
  sprintf(url, "%s", getReq);
  client.setTimeout(10000);
  if (!client.connect("my-json-server.typicode.com", 80)) {
    Serial.println(F("Connection failed"));
    return;
  }
  Serial.println(F("Connected!"));
  // Send HTTP request
  client.println(url);
  client.println(F("Host: my-json-server.typicode.com"));
  client.println("Connection: close");
  client.println();

  if (client.println() == 0) {
    Serial.println(F("Failed to send request"));
    return;
  }
  // Check HTTP status
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    Serial.print(F("Unexpected response: "));
    Serial.println(status);
    return;
  }
  // Skip HTTP headers
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    return;
  }
  // Allocate JsonBuffer
  // Use arduinojson.org/assistant to compute the capacity.
  const size_t capacity = JSON_OBJECT_SIZE(1) + 20;
  DynamicJsonBuffer jsonBuffer(capacity);
  // Parse JSON object
  JsonObject& root = jsonBuffer.parseObject(client);
  if (!root.success()) {
    Serial.println(F("Parsing failed!"));
    return;
  }
  client.stop();
  char* response = root["comando"].as<char*>();
  if (strcmp(response, "R") == 0 ) {
    acionaValvula();
  } else {
    if (strcmp(response, "L") == 0) {
      ligaLuz();
      luzStatus = 1;
    } else {
      if (strcmp(response, "D") == 0) {
      }
      desligaLuz();
      luzStatus = 0;
    }
  }
  solo = -1;
  humidity = -1;
  temperature = -1;
  delay(5000);
}

void configWifi() {
  byte cont = 0;
  while (status != WL_CONNECTED) {
    if (cont > 3) {
      errorInicializacao(3, 200, 500);
    }
    status = WiFi.begin(ssid.c_str(), password.c_str());
    cont++;
  }
}

void musica() {
  buzzer.begin(100);
  buzzer.sound(392, 50);
  buzzer.sound(0, 50);
  buzzer.sound(440, 50);
  buzzer.sound(0, 50);
  buzzer.sound(494, 50);
  buzzer.sound(0, 50);
  buzzer.end(2000);
}
void start() {
  buzzer.begin(100);
  buzzer.sound(392, 50);
  buzzer.sound(0, 50);
  buzzer.end(500);
}

void configParametros()
{
  ssid = configs[0];
  password = configs[1];
}

void ligaLuz() {
  digitalWrite(pin_luz, LOW);
}
void desligaLuz() {
  digitalWrite(pin_luz, HIGH);
}

void acionaValvula() {
  digitalWrite(pin_valvula, LOW);
  delay(1000);
  digitalWrite(pin_valvula, HIGH);
}

int sensorSolo() {
  int valor_solo = constrain(analogRead(sensor_solo), analogSoloMolhado, analogSoloSeco); //MANTÉM valorLido DENTRO DO INTERVALO (ENTRE analogSoloMolhado E analogSoloSeco)
  valor_solo =  map(valor_solo, analogSoloMolhado, analogSoloSeco, percSoloMolhado, percSoloSeco); //EXECUTA A FUNÇÃO "map" DE ACORDO COM OS PARÂMETROS PASSADOS
  return  valor_solo ;
}

void errorInicializacao(int vezes, int frequencia, int del) {
  buzzer.begin(100);
  for (int i = 0; i < vezes; i++) {

    buzzer.sound(frequencia, 800);
    buzzer.sound(0, 80);
    delay(del);
  }
  delay(5000);
  asm volatile ("  jmp 0");
}
