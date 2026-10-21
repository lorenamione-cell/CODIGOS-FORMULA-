const uint8_t PINO_SENSOR = 2;
const uint32_t PPR = 30; // Furos do disco fônico

// Variáveis voláteis compartilhadas com a interrupção
volatile unsigned long g_tempoUltimoPulsoUs = 0;
volatile unsigned long g_periodoPulsoUs = 0;
volatile bool g_novoPulso = false;

// Variável para detectar motor parado
unsigned long g_tempoUltimaLeituraMs = 0;
const unsigned long TIMEOUT_PARADO_MS = 500; // Se ficar 0.5s sem pulso, considera parado

void ISR_sensorOptico() {
  unsigned long tempoAtualUs = micros();
  
  // Evita divisão por zero ou saltos muito curtos (filtro básico de ruído de 100us)
  if (g_tempoUltimoPulsoUs != 0) {
    unsigned long deltaUs = tempoAtualUs - g_tempoUltimoPulsoUs;
    if (deltaUs > 100) { 
      g_periodoPulsoUs = deltaUs;
      g_novoPulso = true;
    }
  }
  g_tempoUltimoPulsoUs = tempoAtualUs;
}

void setup() {
  Serial.begin(115200);
  pinMode(PINO_SENSOR, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(PINO_SENSOR),
    ISR_sensorOptico,
    FALLING
  );

  Serial.println(F("Periodo_us | RPM"));
}

void loop() {
  unsigned long periodoUsLocal = 0;
  bool temDadoNovo = false;

  // Leitura segura das variáveis da interrupção
  noInterrupts();
  if (g_novoPulso) {
    periodoUsLocal = g_periodoPulsoUs;
    g_novoPulso = false;
    temDadoNovo = true;
  }
  interrupts();

  if (temDadoNovo) {
    g_tempoUltimaLeituraMs = millis(); // Atualiza o tempo do último pulso recebido

    // Converte o período entre furos para RPM
    // RPM = (1 / periodo_em_segundos) * (60 / PPR)
    // Como periodoUsLocal está em microssegundos: RPM = (1,000,000 / periodoUsLocal) * (60 / 30)
    float rpm = (1000000.0 / (float)periodoUsLocal) * (60.0 / (float)PPR);

    Serial.print(periodoUsLocal);
    Serial.print(F(" | "));
    Serial.println(rpm, 2);
  }

  // Se o motor parar, zera o RPM no monitor serial após o tempo limite
  if (millis() - g_tempoUltimaLeituraMs > TIMEOUT_PARADO_MS) {
    g_tempoUltimaLeituraMs = millis(); // Evita printar infinitamente
    Serial.println(F("0 | 0.00"));
  }
}
