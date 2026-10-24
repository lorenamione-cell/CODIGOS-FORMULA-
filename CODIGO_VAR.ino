const uint8_t PINO_SENSOR = 2;
const uint32_t PPR = 30; // Furos do disco fônico

// Variáveis voláteis compartilhadas com a interrupção
volatile unsigned long g_tempoUltimoPulsoUs = 0;
volatile unsigned long g_periodoPulsoUs = 0;
volatile bool g_novoPulso = false;
volatile bool g_reiniciarContagem = true; // Flag para resetar o primeiro pulso após parada

// Variável para detectar motor parado
unsigned long g_tempoUltimaLeituraMs = 0;
const unsigned long TIMEOUT_PARADO_MS = 500; // 0.5s sem pulso = parado

void ISR_sensorOptico() {
  unsigned long tempoAtualUs = micros();
  
  // Se o motor estava parado, o próximo pulso é obrigatoriamente o recomeço
  if (g_reiniciarContagem) {
    g_tempoUltimoPulsoUs = tempoAtualUs;
    g_reiniciarContagem = false;
    return;
  }

  unsigned long deltaUs = tempoAtualUs - g_tempoUltimoPulsoUs;
  
  // FILTRO ANTI-RUÍDO RIGOROSO: 
  // Ignora intervalos menores que 2000us (2ms). 
  // Ajuste esse valor caso sua rotação máxima ultrapasse limites teóricos.
  if (deltaUs > 2000) { 
    g_periodoPulsoUs = deltaUs;
    g_novoPulso = true;
    g_tempoUltimoPulsoUs = tempoAtualUs; // Só atualiza se o pulso for válido!
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PINO_SENSOR, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(PINO_SENSOR),
    ISR_sensorOptico,
    FALLING
  );

  Serial.println(F("Periodo_s | RPM"));
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

    // Converte o período para segundos e calcula o RPM
    float periodoSegundos = (float)periodoUsLocal / 1000000.0;
    float rpm = (1.0 / periodoSegundos) * (60.0 / (float)PPR);

    // Imprime o período em segundos e o RPM
    Serial.print(periodoSegundos, 4);
    Serial.print(F(" s | "));
    Serial.println(rpm, 2);
  }

  // Se o motor parar, zera o RPM no monitor serial após o tempo limite e reseta a contagem
  if (millis() - g_tempoUltimaLeituraMs > TIMEOUT_PARADO_MS) {
    noInterrupts();
    g_reiniciarContagem = true; // Força a reiniciar o par de pulsos na próxima rotação
    interrupts();
    
    g_tempoUltimaLeituraMs = millis(); // Evita printar infinitamente
    Serial.println(F("0.0000 s | 0.00"));
  }
}
