#include <Arduino.h>

// =====================================================
// 1. CONFIGURACOES GERAIS
// =====================================================

const uint8_t PINO_SENSOR_1 = 2;  // INT0 - Eixo 1
const uint8_t PINO_SENSOR_2 = 3;  // INT1 - Eixo 2

const uint32_t PPR = 30;          // Pulsos por revolucao

const float RAIO_RODA_M = 0.26f;
const float PI_F = 3.14159265f;

// Perímetro da roda já medido em metros (63 cm = 0.63 metros)
const float PERIMETRO_RODA_M = 0.63f;

// Relacao entre RPM da roda e RPM do eixo medido.
// 1.0 = mesma rotacao.
const float FATOR_ROTACAO_RODA = 1.0f;

// Timeout: 1,5 segundo sem pulsos
const uint32_t TIMEOUT_PARADO_US = 1500000UL;

// Filtro antirruido: intervalo minimo entre pulsos
const uint32_t FILTRO_MIN_US = 200UL;

// Atualizacao serial: 100 ms (aproximadamente 10 Hz)
const uint32_t INTERVALO_SERIAL_MS = 100UL;


// =====================================================
// 2. VARIAVEIS COMPARTILHADAS - EIXO 1
// =====================================================

volatile uint32_t ultimoPulso1 = 0;
volatile uint32_t periodo1 = 0;

volatile bool referencia1 = false;
volatile bool novoDado1 = false;


// =====================================================
// 3. VARIAVEIS COMPARTILHADAS - EIXO 2
// =====================================================

volatile uint32_t ultimoPulso2 = 0;
volatile uint32_t periodo2 = 0;

volatile bool referencia2 = false;
volatile bool novoDado2 = false;


// =====================================================
// 4. INTERRUPCAO DO SENSOR 1
// =====================================================

void ISR_Sensor1() {

    uint32_t agora = micros();

    // Primeiro pulso: estabelece a referencia.
    if (!referencia1) {

        ultimoPulso1 = agora;
        referencia1 = true;

        return;
    }

    // Calcula o intervalo entre pulsos.
    uint32_t delta = agora - ultimoPulso1;

    // Filtro antirruido.
    if (delta >= FILTRO_MIN_US) {

        ultimoPulso1 = agora;
        periodo1 = delta;

        novoDado1 = true;
    }
}


// =====================================================
// 5. INTERRUPCAO DO SENSOR 2
// =====================================================

void ISR_Sensor2() {

    uint32_t agora = micros();

    // Primeiro pulso: estabelece a referencia.
    if (!referencia2) {

        ultimoPulso2 = agora;
        referencia2 = true;

        return;
    }

    // Calcula o intervalo entre pulsos.
    uint32_t delta = agora - ultimoPulso2;

    // Filtro antirruido.
    if (delta >= FILTRO_MIN_US) {

        ultimoPulso2 = agora;
        periodo2 = delta;

        novoDado2 = true;
    }
}


// =====================================================
// 6. VARIAVEIS DO PROGRAMA PRINCIPAL
// =====================================================

// Grandezas calculadas
float rpm1 = 0.0f;
float rpm2 = 0.0f;

float velocidade1 = 0.0f;
float velocidade2 = 0.0f;

// Estados de funcionamento
bool eixo1Ativo = false;
bool eixo2Ativo = false;

// Indica se ja existe uma medicao valida
bool medicaoValida1 = false;
bool medicaoValida2 = false;

// Historico dos estados para detectar mudancas
bool estadoAnterior1 = false;
bool estadoAnterior2 = false;

// Controle da impressao serial
uint32_t ultimaImpressaoMs = 0;


// =====================================================
// 7. CONFIGURACAO INICIAL
// =====================================================

void setup() {

    Serial.begin(115200);

    pinMode(PINO_SENSOR_1, INPUT_PULLUP);
    pinMode(PINO_SENSOR_2, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(PINO_SENSOR_1),
        ISR_Sensor1,
        FALLING
    );

    attachInterrupt(
        digitalPinToInterrupt(PINO_SENSOR_2),
        ISR_Sensor2,
        FALLING
    );

    Serial.println(F("=========================================="));
    Serial.println(F("       SISTEMA DE MEDICAO - FORMULA SAE"));
    Serial.println(F("=========================================="));

    Serial.println(
        F("EIXO 1 (RPM | km/h) || EIXO 2 (RPM | km/h)")
    );
}


// =====================================================
// 8. LOOP PRINCIPAL
// =====================================================

void loop() {

    // -------------------------------------------------
    // 8.1 VARIAVEIS LOCAIS PARA AQUISICAO
    // -------------------------------------------------

    uint32_t p1 = 0;
    uint32_t p2 = 0;

    uint32_t agoraUs;

    bool temDado1 = false;
    bool temDado2 = false;

    bool ativoLocal1 = false;
    bool ativoLocal2 = false;

    bool houveParada1 = false;
    bool houveParada2 = false;


    // -------------------------------------------------
    // 8.2 LEITURA ATOMICA E CONTROLE DE TIMEOUT
    // -------------------------------------------------

    noInterrupts();

    agoraUs = micros();

    // Captura dos dados do eixo 1
    if (novoDado1) {

        p1 = periodo1;
        temDado1 = true;

        novoDado1 = false;
    }

    // Captura dos dados do eixo 2
    if (novoDado2) {

        p2 = periodo2;
        temDado2 = true;

        novoDado2 = false;
    }

    // Verificacao de parada do eixo 1
    if (referencia1 &&
        (uint32_t)(agoraUs - ultimoPulso1) >= TIMEOUT_PARADO_US) {

        referencia1 = false;
        novoDado1 = false;

        houveParada1 = true;
    }

    // Verificacao de parada do eixo 2
    if (referencia2 &&
        (uint32_t)(agoraUs - ultimoPulso2) >= TIMEOUT_PARADO_US) {

        referencia2 = false;
        novoDado2 = false;

        houveParada2 = true;
    }

    // Captura dos estados atuais
    ativoLocal1 = referencia1;
    ativoLocal2 = referencia2;

    interrupts();


    // -------------------------------------------------
    // 8.3 PROCESSAMENTO DO EIXO 1
    // -------------------------------------------------

    if (houveParada1 || !ativoLocal1) {

        rpm1 = 0.0f;
        velocidade1 = 0.0f;

        medicaoValida1 = false;
    }

    else if (temDado1 && p1 > 0) {

        rpm1 = 60000000.0f /
               ((float)PPR * (float)p1);

        velocidade1 = rpm1 * FATOR_ROTACAO_RODA * PERIMETRO_RODA_M * 60.0f / 1000.0f;

        medicaoValida1 = true;
    }

    eixo1Ativo = ativoLocal1;


    // -------------------------------------------------
    // 8.4 PROCESSAMENTO DO EIXO 2
    // -------------------------------------------------

    if (houveParada2 || !ativoLocal2) {

        rpm2 = 0.0f;
        velocidade2 = 0.0f;

        medicaoValida2 = false;
    }

    else if (temDado2 && p2 > 0) {

        rpm2 = 60000000.0f /
               ((float)PPR * (float)p2);

        velocidade2 =
            rpm2 *
            FATOR_ROTACAO_RODA *
            PERIMETRO_RODA_M *
            60.0f / 1000.0f;

        medicaoValida2 = true;
    }

    eixo2Ativo = ativoLocal2;


    // -------------------------------------------------
    // 8.5 DETECCAO DE MUDANCA DE ESTADO
    // -------------------------------------------------

    bool mudouEstado =
        (eixo1Ativo != estadoAnterior1) ||
        (eixo2Ativo != estadoAnterior2);


    // -------------------------------------------------
    // 8.6 CONTROLE DA FREQUENCIA DE IMPRESSAO
    // -------------------------------------------------

    uint32_t agoraMs = millis();

    bool chegouIntervalo =
        (uint32_t)(agoraMs - ultimaImpressaoMs)
        >= INTERVALO_SERIAL_MS;


    // Imprime durante a atividade ou quando ocorre
    // uma transicao entre ativo e parado.

    if ((eixo1Ativo || eixo2Ativo) &&
        (chegouIntervalo || mudouEstado)) {

        // ---------------- EIXO 1 ----------------

        Serial.print(F("E1: "));

        if (!eixo1Ativo) {

            Serial.print(F("0.0 RPM | 0.0 km/h"));
        }

        else if (!medicaoValida1) {

            Serial.print(F("AGUARDANDO PULSOS"));
        }

        else {

            Serial.print(rpm1, 1);
            Serial.print(F(" RPM | "));
            Serial.print(velocidade1, 1);
            Serial.print(F(" km/h"));
        }

        Serial.print(F(" || "));

        // ---------------- EIXO 2 ----------------

        Serial.print(F("E2: "));

        if (!eixo2Ativo) {

            Serial.print(F("0.0 RPM | 0.0 km/h"));
        }

        else if (!medicaoValida2) {

            Serial.print(F("AGUARDANDO PULSOS"));
        }

        else {

            Serial.print(rpm2, 1);
            Serial.print(F(" RPM | "));
            Serial.print(velocidade2, 1);
            Serial.print(F(" km/h"));
        }

        Serial.println();

        ultimaImpressaoMs = agoraMs;
    }

    // Quando ambos param, imprime zero somente uma vez.
    else if (!eixo1Ativo &&
             !eixo2Ativo &&
             mudouEstado) {

        Serial.println(
            F("E1: 0.0 RPM | 0.0 km/h || E2: 0.0 RPM | 0.0 km/h")
        );

        ultimaImpressaoMs = agoraMs;
    }


    // -------------------------------------------------
    // 8.7 ATUALIZACAO DOS ESTADOS ANTERIORES
    // -------------------------------------------------

    estadoAnterior1 = eixo1Ativo;
    estadoAnterior2 = eixo2Ativo;
}
