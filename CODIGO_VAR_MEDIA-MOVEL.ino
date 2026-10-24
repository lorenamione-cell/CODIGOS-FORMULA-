const uint8_t PINO_SENSOR = 2;
const uint32_t PPR = 30; // Furos do disco fônico

// Variáveis voláteis compartilhadas com a interrupção
volatile unsigned long g_tempoUltimoPulsoUs = 0;
volatile unsigned long g_periodoPulsoUs = 0;
volatile bool g_novoPulso = false;
volatile bool g_reiniciarContagem = true;

// Variável para detectar motor parado
unsigned long g_tempoUltimaLeituraMs = 0;
const unsigned long TIMEOUT_PARADO_MS = 500; // 0.5s sem pulso = parado

// Configuração da Média Móvel
const uint8_t TAMANHO_FILTRO = 5; // Quantidade de amostras para tirar a média
float g_historicoRpm[TAMANHO_FILTRO] = {0};
uint8_t g_indiceFiltro = 0;
bool g_filtroCheio = false;

void ISR_sensorOptico() {
  unsigned long tempoAtualUs = micros();
  
  if (g_reiniciarContagem) {
    g_tempoUltimoPulsoUs = tempoAtualUs;
    g_reiniciarContagem = false;
    return;
  }

  unsigned long deltaUs = tempoAtualUs - g_tempoUltimoPulsoUs;
  
  // Filtro anti-ruído (2ms)
  if (deltaUs > 2000) { 
    g_periodoPulsoUs = deltaUs;
    g_novoPulso = true;
    g_tempoUltimoPulsoUs = tempoAtualUs;
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

  Serial.println(F("Periodo_s | RPM (Suavizado)"));
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

    // Converte o período para segundos e calcula o RPM instantâneo
    float periodoSegundos = (float)periodoUsLocal / 1000000.0;
    float rpmInstantaneo = (1.0 / periodoSegundos) * (60.0 / (float)PPR);

    // --- APLICAÇÃO DA MÉDIA MÓVEL ---
    g_historicoRpm[g_indiceFiltro] = rpmInstantaneo;
    g_indiceFiltro = (g_indiceFiltro + 1) % TAMANHO_FILTRO;
    if (g_indiceFiltro == 0) {
      g_filtroCheio = true;
    }

    // Soma os valores válidos armazenados no histórico
    uint8_t quantidadeElementos = g_filtroCheio ? TAMANHO_FILTRO : g_indiceFiltro;
    float somaRpm = 0;
    for (uint8_t i = 0; i < quantidadeElementos; i++) {
      somaRpm += g_historicoRpm[i];
    }
    
    // Calcula a média final
    float rpmMedio = somaRpm / quantidadeElementos;
    // --------------------------------

    // Imprime o período e o RPM suavizado
    Serial.print(periodoSegundos, 4);
    Serial.print(F(" s | "));
    Serial.println(rpmMedio, 2);
  }

  // Se o motor parar, zera o RPM e limpa o histórico da média móvel
  if (millis() - g_tempoUltimaLeituraMs > TIMEOUT_PARADO_MS) {
    noInterrupts();
    g_reiniciarContagem = true;
    
    // Reseta o filtro para evitar resíduos de velocidade anterior
    for (uint8_t i = 0; i < TAMANHO_FILTRO; i++) {
      g_historicoRpm[i] = 0;
    }
    g_indiceFiltro = 0;
    g_filtroCheio = false;
    interrupts();
    
    g_tempoUltimaLeituraMs = millis(); 
    Serial.println(F("0.0000 s | 0.00"));
  }
}
