/* controle proporcional de tensão PWM para automação do Projeto:
*  MMO-MICRO MAÇARICO OXÍDRICO - retrofit
Placa ESP32S3 Dev Module
____________________________________________________________________
*## **Correções Principais:**

1. **Debounce do botão**: Adicionei sistema de debounce para evitar múltiplas leituras acidentais do botão
2. **Toggle do sistema**: Agora o botão liga/desliga o sistema ao invés de precisar manter pressionado
3. **Inicialização adequada**: Adicionei pinMode para todas as entradas analógicas
4. **Controle de tempo**: Uso de millis() para controle não-bloqueante das atualizações
5. **Display organizado**: Melhorei o layout do LCD para mostrar informações mais claras
6. **Debug serial**: Adicionei comunicação serial para facilitar o debug
7. **Inicialização do PWM**: PWM começa em 0 para segurança
8. **Estrutura mais robusta**: Separei as funções em blocos mais organizados

## **Melhorias Implementadas:**

- **Segurança**: Sistema inicia desligado e só ativa quando solicitado
- **Estabilidade**: Controle de tempo evita travamentos
- **Usabilidade**: Interface mais clara no LCD
- **Manutenibilidade**: Código mais organizado e comentado
- **Debug**: Saída serial para monitoramento
_______________________________________________________________________ */

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// =================== Verificação da placa ESP32-S3 ===================
#ifndef CONFIG_IDF_TARGET_ESP32S3
#error "Este código foi desenvolvido especificamente para ESP32-S3"
#endif

// =================== LCD com I2C padrão ===================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD de 16 colunas por 2 linhas

// =================== Definição de pinos para ESP32-S3 ===================
#define PIN_CORRENTE 1     // GPIO1 - Sensor de corrente (ADC1_CH0)
#define PIN_PRESSAO 2      // GPIO2 - Sensor de pressão (ADC1_CH1)
#define PIN_BOTAO_START 4  // GPIO4 - Botão de Start
#define PIN_SCR_PWM 18     // GPIO18 - Saída PWM para gate do SCR
#define PIN_SOLENOIDE 15   // GPIO15 - Controle da válvula solenóide
#define PIN_LCD_SDA 8      // GPIO8 - Pino SDA do LCD I2C
#define PIN_LCD_SCL 9      // GPIO9 - Pino SCL do LCD I2C

// =================== Parâmetros PWM para ESP32-S3 ===================
#define PWM_CANAL 0
#define PWM_FREQ 5000      // Frequência 5kHz
#define PWM_RES_BITS 8     // 8 bits: valores de 0 a 255
#define PWM_MAX_VALUE 255  // Valor máximo PWM

// =================== Resolução ADC ESP32-S3 ===================
#define ADC_RESOLUTION 4095.0f  // 12 bits (0-4095)

// =================== Parâmetros do controle ===================
float correnteSetpoint = 25.0;  // Corrente alvo (em A)
float kp = 10.0;                // Ganho proporcional (valor empírico)
float pwmBase = 100.0;          // Base PWM ajustável

float pressaoMinima = 0.9;  // Faixa de pressão segura mínima (bar)
float pressaoMaxima = 1.2;  // Faixa de pressão segura máxima (bar)

// =================== Variáveis para debounce do botão ===================
unsigned long ultimoDebounce = 0;
const unsigned long debounceDelay = 50;
bool estadoAnteriorBotao = HIGH;
bool estadoAtualBotao = HIGH;
bool sistemaAtivo = false;

// =================== Função de leitura de corrente ===================
float lerCorrente() {
  int leitura = analogRead(PIN_CORRENTE);
  return (leitura / ADC_RESOLUTION) * 50.0f;  // Conversão para 0–50A
}

// =================== Função de leitura de pressão ===================
float lerPressao() {
  int leitura = analogRead(PIN_PRESSAO);
  return (leitura / ADC_RESOLUTION) * 2.0f;  // Conversão para 0–2 bar
}

// =================== Controle proporcional via PWM ===================
void controlarTensao(float correnteAtual) {
  float erro = correnteSetpoint - correnteAtual;
  float correcao = kp * erro;
  float pwm = pwmBase + correcao;
  pwm = constrain(pwm, 0, 255);
  ledcWrite(PWM_CANAL, (int)pwm);
}

// =================== Função para debounce do botão ===================
bool lerBotaoComDebounce() {
  bool leitura = digitalRead(PIN_BOTAO_START);

  if (leitura != estadoAnteriorBotao) {
    ultimoDebounce = millis();
  }

  if ((millis() - ultimoDebounce) > debounceDelay) {
    if (leitura != estadoAtualBotao) {
      estadoAtualBotao = leitura;
      if (estadoAtualBotao == LOW) {
        sistemaAtivo = !sistemaAtivo;  // Toggle do estado
      }
    }
  }

  estadoAnteriorBotao = leitura;
  return sistemaAtivo;
}

// =================== Função para atualizar display LCD ===================
void atualizarDisplay(float corrente, float pressao, bool ativo) {
  if (ativo) {
    lcd.setCursor(0, 0);
    lcd.print("I:");
    lcd.print(corrente, 1);
    lcd.print("A  P:");
    lcd.print(pressao, 1);
    lcd.print("B");

    lcd.setCursor(0, 1);
    lcd.print("Sistema ATIVO   ");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Sistema PARADO  ");
    lcd.setCursor(0, 1);
    lcd.print("Pressione START ");
  }
}

void setup() {
  // Inicialização da comunicação serial para debug
  Serial.begin(115200);
  Serial.println("Iniciando ESP32-S3 Eletrolisador...");

  // Configuração da resolução ADC para ESP32-S3
  analogReadResolution(12);  // 12 bits para ESP32-S3

  // Inicialização I2C com pinos específicos do ESP32-S3
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();

  // Inicialização dos pinos
  pinMode(PIN_CORRENTE, INPUT);
  pinMode(PIN_PRESSAO, INPUT);
  pinMode(PIN_BOTAO_START, INPUT_PULLUP);
  pinMode(PIN_SOLENOIDE, OUTPUT);
  digitalWrite(PIN_SOLENOIDE, LOW);

  // Configuração PWM para ESP32-S3
  ledcSetup(PWM_CANAL, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_SCR_PWM, PWM_CANAL);
  ledcWrite(PWM_CANAL, 0);  // Inicializa PWM em 0

  // Mensagem de boas-vindas
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32-S3");
  lcd.setCursor(0, 1);
  lcd.print("Eletrolisador...");
  delay(2000);
  lcd.clear();

  Serial.println("ESP32-S3 inicializado com sucesso!");
  Serial.print("Frequência da CPU: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
}

void loop() {
  static bool valvulaAberta = false;
  static unsigned long ultimaAtualizacao = 0;
  const unsigned long intervaloAtualizacao = 500;

  // Leitura do botão com debounce
  bool sistemaDeveLigar = lerBotaoComDebounce();

  // Atualização periódica das leituras
  if (millis() - ultimaAtualizacao >= intervaloAtualizacao) {
    float corrente = lerCorrente();
    float pressao = lerPressao();

    if (sistemaDeveLigar) {
      // Sistema ativo - executar controle
      controlarTensao(corrente);

      // Controle da válvula com histerese
      if (pressao >= pressaoMaxima && !valvulaAberta) {
        digitalWrite(PIN_SOLENOIDE, HIGH);
        valvulaAberta = true;
        Serial.println("Válvula ABERTA - Pressão alta");
      } else if (pressao <= pressaoMinima && valvulaAberta) {
        digitalWrite(PIN_SOLENOIDE, LOW);
        valvulaAberta = false;
        Serial.println("Válvula FECHADA - Pressão normalizada");
      }

      // Debug via serial
      Serial.print("Corrente: ");
      Serial.print(corrente);
      Serial.print("A, Pressão: ");
      Serial.print(pressao);
      Serial.println("bar");

    } else {
      // Sistema desativo - desligar tudo
      digitalWrite(PIN_SOLENOIDE, LOW);
      ledcWrite(PWM_CANAL, 0);
      valvulaAberta = false;
    }

    // Atualizar display
    atualizarDisplay(corrente, pressao, sistemaDeveLigar);

    ultimaAtualizacao = millis();
  }

  // Pequeno delay para não sobrecarregar o processador
  delay(10);
}
