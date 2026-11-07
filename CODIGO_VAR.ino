
#include <Arduino.h>

// ============================================================
// SISTEMA DE MEDICAO DE ROTACAO - FORMULA SAE
// Arduino Nano classico (ATmega328P)
// Dois sensores opticos, um por eixo
// ============================================================

// ------------------------------------------------------------
// 1. CONFIGURACOES
// ------------------------------------------------------------

// Pinos dos sensores
const uint8_t PINO_SENSOR_1 = 2;
const uint8_t PINO_SENSOR_2 = 3;

// Disco fonico
const uint16_t PPR = 30;  // 30 aberturas = 30 pulsos por volta

// Borda de deteccao.
// RISING: transicao LOW -> HIGH
// FALLING: transicao HIGH -> LOW
// RISING foi a configuracao anteriormente testada.
const int BORDA_SENSOR = RISING;

// Entrada eletrica:
// INPUT nao ativa o pull-up interno de 5 V.
// A saida do sensor precisa ter nivel logico definido
// por circuito push-pull ou pull-up externo adequado.
const uint8_t MODO_ENTRADA = INPUT;

// Filtro contra pulsos muito proximos
const uint32_t FILTRO_MIN_US = 200UL;

// Tempo sem pulsos para considerar o eixo parado
const uint32_t TIMEOUT_PARADO_US = 2000000UL;

// Intervalos de atualizacao
const uint32_t INTERVALO_SERIAL_MS = 100UL;

// ------------------------------------------------------------
// 2. VELOCIDADE
// ------------------------------------------------------------

// Deixe 0 ate medir a circunferencia real da roda.
#define HABILITAR_VELOCIDADE 0

// Parametros provisórios. Nao serao usados enquanto
// HABILITAR_VELOCIDADE estiver em 0.
const float PERIMETRO_RODA_M = 0.63f;
const float FATOR_ROTACAO_RODA = 1.0f;

// ------------------------------------------------------------
// 3. CARTAO SD
// ------------------------------------------------------------

// Deixe 0 enquanto o modulo SD nao estiver instalado.
// Altere para 1 depois de conectar e verificar o modulo.
#define HABILITAR_SD 0

#if HABILITAR_SD
  #include <SPI.h>
  #include <SD.h>

  const uint8_t PINO_CS_SD = 4;
  const uint8_t PINO_SPI_SS = 10;

  const uint32_t INTERVALO_SD_MS = 250UL;
  const uint32_t INTERVALO_FLUSH_MS = 2000UL;

  File arquivoLog;
  bool sdDisponivel = false;

  uint32_t ultimaGravacaoSD = 0;
  uint32_t ultimoFlushSD = 0;
#endif

// ------------------------------------------------------------
// 4. DADOS COMPARTILHADOS COM AS INTERRUPCOES
// ------------------------------------------------------------

// Eixo 1
volatile uint32_t ultimoPulso1 = 0;
volatile uint32_t periodo1 = 0;
volatile bool temReferencia1 = false;
volatile bool periodoValido1 = false;
volatile bool jaRecebeuPulso1 = false;

// Eixo 2
volatile uint32_t ultimoPulso2 = 0;
volatile uint32_t periodo2 = 0;
volatile bool temReferencia2 = false;
volatile bool periodoValido2 = false;
volatile bool jaRecebeuPulso2 = false;

// ------------------------------------------------------------
// 5. ESTADOS
// ------------------------------------------------------------

enum EstadoEixo {
  SEM_SINAL,
  AGUARDANDO,
  ATIVO,
  PARADO
};

EstadoEixo estado1 = SEM_SINAL;
EstadoEixo estado2 = SEM_SINAL;

EstadoEixo estadoAnterior1 = SEM_SINAL;
EstadoEixo estadoAnterior2 = SEM_SINAL;

// ------------------------------------------------------------
// 6. MEDIDAS
// ------------------------------------------------------------

float rpm1 = 0.0f;
float rpm2 = 0.0f;

float velocidade1 = 0.0f;
float velocidade2 = 0.0f;

uint32_t ultimaImpressaoSerial = 0;

// ------------------------------------------------------------
// 7. INTERRUPCAO - EIXO 1
// ------------------------------------------------------------

void ISR_Sensor1() {

  const uint32_t agora = micros();

  jaRecebeuPulso1 = true;

  // Primeiro pulso: estabelece a referencia temporal
  if (!temReferencia1) {
    ultimoPulso1 = agora;
    temReferencia1 = true;
    periodoValido1 = false;
    return;
  }

  const uint32_t delta = agora - ultimoPulso1;

  // Ignora pulsos muito proximos
  if (delta < FILTRO_MIN_US) {
    return;
  }

  ultimoPulso1 = agora;
  periodo1 = delta;
  periodoValido1 = true;
}

// ------------------------------------------------------------
// 8. INTERRUPCAO - EIXO 2
// ------------------------------------------------------------

void ISR_Sensor2() {

  const uint32_t agora = micros();

  jaRecebeuPulso2 = true;

  if (!temReferencia2) {
    ultimoPulso2 = agora;
    temReferencia2 = true;
    periodoValido2 = false;
    return;
  }

  const uint32_t delta = agora - ultimoPulso2;

  if (delta < FILTRO_MIN_US) {
    return;
  }

  ultimoPulso2 = agora;
  periodo2 = delta;
  periodoValido2 = true;
}

// ------------------------------------------------------------
// 9. NOME DOS ESTADOS
// ------------------------------------------------------------

const char* nomeEstado(EstadoEixo estado) {

  switch (estado) {

    case SEM_SINAL:
      return "SEM_SINAL";

    case AGUARDANDO:
      return "AGUARDANDO";

    case ATIVO:
      return "ATIVO";

    case PARADO:
      return "PARADO";
  }

  return "ERRO";
}

// ------------------------------------------------------------
// 10. CALCULO DE RPM
// ------------------------------------------------------------

float calcularRPM(uint32_t periodo) {

  if (periodo == 0 || PPR == 0) {
    return 0.0f;
  }

  return 60000000.0f /
         ((float)PPR * (float)periodo);
}

// ------------------------------------------------------------
// 11. CALCULO DE VELOCIDADE
// ------------------------------------------------------------

float calcularVelocidade(float rpm) {

#if HABILITAR_VELOCIDADE

  return rpm *
         FATOR_ROTACAO_RODA *
         PERIMETRO_RODA_M *
         60.0f / 1000.0f;

#else

  return 0.0f;

#endif
}

// ------------------------------------------------------------
// 12. INICIALIZACAO DO SD
// ------------------------------------------------------------

#if HABILITAR_SD

void iniciarSD() {

  pinMode(PINO_SPI_SS, OUTPUT);
  digitalWrite(PINO_SPI_SS, HIGH);

  pinMode(PINO_CS_SD, OUTPUT);
  digitalWrite(PINO_CS_SD, HIGH);

  SPI.begin();

  Serial.print(F("Inicializando SD... "));

  if (!SD.begin(PINO_CS_SD)) {
    Serial.println(F("FALHA"));
    sdDisponivel = false;
    return;
  }

  arquivoLog = SD.open("RPMLOG.CSV", FILE_WRITE);

  if (!arquivoLog) {
    Serial.println(F("ERRO AO ABRIR ARQUIVO"));
    sdDisponivel = false;
    return;
  }

  // Cria cabecalho somente se o arquivo estiver vazio
  if (arquivoLog.size() == 0) {

    arquivoLog.println(
      F("Tempo_ms,"
        "Estado1,Periodo1_us,RPM1,Vel1_kmh,"
        "Estado2,Periodo2_us,RPM2,Vel2_kmh")
    );

    arquivoLog.flush();

    if (arquivoLog.getWriteError()) {
      Serial.println(F("ERRO NO CABECALHO"));
      arquivoLog.close();
      sdDisponivel = false;
      return;
    }
  }

  sdDisponivel = true;

  Serial.println(F("SD OK"));
  Serial.println(F("Arquivo: RPMLOG.CSV"));
}

#endif

// ------------------------------------------------------------
// 13. IMPRESSAO SERIAL
// ------------------------------------------------------------

void imprimirEixo(
  const char* nome,
  EstadoEixo estado,
  float rpm,
  float velocidade
) {

  Serial.print(nome);
  Serial.print(F(": "));
  Serial.print(nomeEstado(estado));

  if (estado == ATIVO) {

    Serial.print(F(" | "));
    Serial.print(rpm, 2);
    Serial.print(F(" RPM"));

#if HABILITAR_VELOCIDADE

    Serial.print(F(" | "));
    Serial.print(velocidade, 2);
    Serial.print(F(" km/h"));

#else

    Serial.print(F(" | Velocidade: N/D"));

#endif

  }

  else if (estado == PARADO) {
    Serial.print(F(" | 0 RPM"));
  }
}

// ------------------------------------------------------------
// 14. GRAVACAO NO SD
// ------------------------------------------------------------

#if HABILITAR_SD

void gravarSD(
  uint32_t tempo,
  EstadoEixo e1,
  uint32_t p1,
  float r1,
  float v1,
  EstadoEixo e2,
  uint32_t p2,
  float r2,
  float v2
) {

  if (!sdDisponivel || !arquivoLog) {
    return;
  }

  arquivoLog.print(tempo);
  arquivoLog.print(",");

  arquivoLog.print(nomeEstado(e1));
  arquivoLog.print(",");

  if (e1 == ATIVO) {
    arquivoLog.print(p1);
    arquivoLog.print(",");
    arquivoLog.print(r1, 2);
    arquivoLog.print(",");
#if HABILITAR_VELOCIDADE
    arquivoLog.print(v1, 2);
#else
    arquivoLog.print("");
#endif
  } else {
    arquivoLog.print(",");
    arquivoLog.print((e1 == PARADO) ? "0.00" : "");
    arquivoLog.print(",");
    arquivoLog.print((e1 == PARADO) ? "0.00" : "");
  }

  arquivoLog.print(",");
  arquivoLog.print(nomeEstado(e2));
  arquivoLog.print(",");

  if (e2 == ATIVO) {
    arquivoLog.print(p2);
    arquivoLog.print(",");
    arquivoLog.print(r2, 2);
    arquivoLog.print(",");
#if HABILITAR_VELOCIDADE
    arquivoLog.print(v2, 2);
#else
    arquivoLog.print("");
#endif
  } else {
    arquivoLog.print(",");
    arquivoLog.print((e2 == PARADO) ? "0.00" : "");
    arquivoLog.print(",");
    arquivoLog.print((e2 == PARADO) ? "0.00" : "");
  }

  arquivoLog.println();

  if (arquivoLog.getWriteError()) {
    Serial.println(F("ERRO DE GRAVACAO SD"));
    arquivoLog.close();
    sdDisponivel = false;
  }
}

#endif

// ------------------------------------------------------------
// 15. SETUP
// ------------------------------------------------------------

void setup() {

  Serial.begin(115200);

  pinMode(PINO_SENSOR_1, MODO_ENTRADA);
  pinMode(PINO_SENSOR_2, MODO_ENTRADA);

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F(" SISTEMA RPM - FORMULA SAE"));
  Serial.println(F(" ARDUINO NANO"));
  Serial.println(F("================================"));

  Serial.print(F("PPR: "));
  Serial.println(PPR);

  Serial.print(F("Filtro minimo (us): "));
  Serial.println(FILTRO_MIN_US);

  Serial.print(F("Timeout (us): "));
  Serial.println(TIMEOUT_PARADO_US);

#if HABILITAR_VELOCIDADE
  Serial.println(F("Velocidade: HABILITADA"));
#else
  Serial.println(F("Velocidade: DESABILITADA"));
#endif

#if HABILITAR_SD
  iniciarSD();
#else
  Serial.println(F("SD: DESABILITADO NO CODIGO"));
#endif

  attachInterrupt(
    digitalPinToInterrupt(PINO_SENSOR_1),
    ISR_Sensor1,
    BORDA_SENSOR
  );

  attachInterrupt(
    digitalPinToInterrupt(PINO_SENSOR_2),
    ISR_Sensor2,
    BORDA_SENSOR
  );

  Serial.println(F("Sensores configurados"));
  Serial.println(F("Aguardando pulsos..."));
}

// ------------------------------------------------------------
// 16. LOOP PRINCIPAL
// ------------------------------------------------------------

void loop() {

  uint32_t agoraUs;

  uint32_t p1, p2;
  uint32_t ultimo1, ultimo2;

  bool referencia1Local;
  bool referencia2Local;

  bool valido1, valido2;
  bool recebeu1, recebeu2;

  // Leitura atomica das variaveis compartilhadas
  noInterrupts();

  agoraUs = micros();

  // Verifica timeout do eixo 1
  if (
    temReferencia1 &&
    (uint32_t)(agoraUs - ultimoPulso1) >= TIMEOUT_PARADO_US
  ) {
    temReferencia1 = false;
    periodoValido1 = false;
  }

  // Verifica timeout do eixo 2
  if (
    temReferencia2 &&
    (uint32_t)(agoraUs - ultimoPulso2) >= TIMEOUT_PARADO_US
  ) {
    temReferencia2 = false;
    periodoValido2 = false;
  }

  p1 = periodo1;
  p2 = periodo2;

  ultimo1 = ultimoPulso1;
  ultimo2 = ultimoPulso2;

  referencia1Local = temReferencia1;
  referencia2Local = temReferencia2;

  valido1 = periodoValido1;
  valido2 = periodoValido2;

  recebeu1 = jaRecebeuPulso1;
  recebeu2 = jaRecebeuPulso2;

  interrupts();

  // ----------------------------------------------------------
  // IDENTIFICACAO DOS ESTADOS
  // ----------------------------------------------------------

  if (!recebeu1) {
    estado1 = SEM_SINAL;
  }
  else if (!referencia1Local) {
    estado1 = PARADO;
  }
  else if (!valido1) {
    estado1 = AGUARDANDO;
  }
  else {
    estado1 = ATIVO;
  }

  if (!recebeu2) {
    estado2 = SEM_SINAL;
  }
  else if (!referencia2Local) {
    estado2 = PARADO;
  }
  else if (!valido2) {
    estado2 = AGUARDANDO;
  }
  else {
    estado2 = ATIVO;
  }

  // ----------------------------------------------------------
  // CALCULO DAS GRANDEZAS
  // ----------------------------------------------------------

  rpm1 = (estado1 == ATIVO) ? calcularRPM(p1) : 0.0f;
  rpm2 = (estado2 == ATIVO) ? calcularRPM(p2) : 0.0f;

  velocidade1 = calcularVelocidade(rpm1);
  velocidade2 = calcularVelocidade(rpm2);

  // ----------------------------------------------------------
  // SERIAL
  // ----------------------------------------------------------

  const uint32_t agoraMs = millis();

  const bool mudouEstado =
    (estado1 != estadoAnterior1) ||
    (estado2 != estadoAnterior2);

  const bool intervaloSerial =
    (uint32_t)(agoraMs - ultimaImpressaoSerial) >=
    INTERVALO_SERIAL_MS;

  if (
    mudouEstado ||
    (intervaloSerial &&
     (estado1 == ATIVO || estado2 == ATIVO))
  ) {

    imprimirEixo("E1", estado1, rpm1, velocidade1);

    Serial.print(F(" || "));

    imprimirEixo("E2", estado2, rpm2, velocidade2);

    Serial.println();

    ultimaImpressaoSerial = agoraMs;
  }

  // ----------------------------------------------------------
  // CARTAO SD
  // ----------------------------------------------------------

#if HABILITAR_SD

  const bool existeMedicao =
    (estado1 != SEM_SINAL) ||
    (estado2 != SEM_SINAL);

  const bool intervaloSD =
    (uint32_t)(agoraMs - ultimaGravacaoSD) >=
    INTERVALO_SD_MS;

  if (sdDisponivel && existeMedicao && intervaloSD) {

    gravarSD(
      agoraMs,
      estado1, p1, rpm1, velocidade1,
      estado2, p2, rpm2, velocidade2
    );

    ultimaGravacaoSD = agoraMs;
  }

  // Flush periodico, sem forcar gravacao a cada linha
  if (
    sdDisponivel &&
    (uint32_t)(agoraMs - ultimoFlushSD) >=
    INTERVALO_FLUSH_MS
  ) {

    arquivoLog.flush();
    ultimoFlushSD = agoraMs;

    if (arquivoLog.getWriteError()) {
      Serial.println(F("ERRO NO FLUSH DO SD"));
      arquivoLog.close();
      sdDisponivel = false;
    }
  }

#endif

  estadoAnterior1 = estado1;
  estadoAnterior2 = estado2;
}
