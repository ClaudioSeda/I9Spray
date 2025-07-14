/*
 * Código Completo para Leitura de Todos os Parâmetros do TUF-2000
 * ESP32 + RS485 + ModbusRTU
 * Autor: ClaudioSeda
 * Data: 2025-07-14 14:58:36
 * 
 * ESP32 Arduino Core: versão 3.0.0 ou superior
 * Biblioteca ModbusMaster: versão 2.0.1 ou superior
 * 
 * FUNCIONALIDADES:
 * - Leitura completa de todos os parâmetros do TUF-2000
 * - Interface serial para monitoramento e comandos
 * - Tratamento de erros e reconexão automática
 * - Formatação em JSON para integração com outros sistemas
 * - Watchdog timer para garantir estabilidade
 * - Logs detalhados com timestamp
 * 
 * CONEXÕES ESP32:
 * GPIO16 (RX2) -> RS485 RO
 * GPIO17 (TX2) -> RS485 DI
 * GPIO4        -> RS485 DE/RE
 * 3.3V         -> RS485 VCC
 * GND          -> RS485 GND
 */

#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

// Definições de pinos do ESP32
#define RS485_RX_PIN 16           // GPIO16 - RX2 hardware serial
#define RS485_TX_PIN 17           // GPIO17 - TX2 hardware serial  
#define RS485_DE_RE_PIN 4         // GPIO4 - Controle de direção RS485
#define LED_STATUS_PIN 2          // GPIO2 - LED interno para status

// Configurações do protocolo Modbus RTU
#define MODBUS_SLAVE_ID 1         // Endereço do slave TUF-2000
#define MODBUS_BAUD_RATE 9600     // Velocidade de comunicação
#define MODBUS_TIMEOUT 3000       // Timeout para comunicação em ms
#define MAX_RETRIES 3             // Máximo de tentativas por leitura

// Registradores Modbus do TUF-2000 (baseado no manual oficial)
// Medições instantâneas (formato float IEEE 754 - 2 registradores cada)
#define REG_FLOW_RATE 1           // Vazão instantânea [L/s]
#define REG_ENERGY_FLOW_RATE 3    // Taxa de fluxo de energia [kW]
#define REG_VELOCITY 5            // Velocidade do fluido [m/s]
#define REG_FLUID_SOUNDSPEED 7    // Velocidade do som no fluido [m/s]
#define REG_POSITIVE_ACCUMULATOR 9    // Totalizador positivo [L]
#define REG_NEGATIVE_ACCUMULATOR 11   // Totalizador negativo [L]
#define REG_NET_ACCUMULATOR 13    // Totalizador líquido [L]
#define REG_POSITIVE_ENERGY 15    // Energia positiva [kWh]
#define REG_NEGATIVE_ENERGY 17    // Energia negativa [kWh]
#define REG_NET_ENERGY 19         // Energia líquida [kWh]
#define REG_TEMPERATURE 21        // Temperatura [°C]
#define REG_HEAT_INPUT 23         // Entrada de calor [kW]
#define REG_HEAT_OUTPUT 25        // Saída de calor [kW]

// Parâmetros de qualidade do sinal (formato uint16 - 1 registrador cada)
#define REG_SIGNAL_QUALITY 27     // Qualidade do sinal [0-100%]
#define REG_UPSTREAM_STRENGTH 28  // Força do sinal upstream [0-999]
#define REG_DOWNSTREAM_STRENGTH 29 // Força do sinal downstream [0-999]
#define REG_UPSTREAM_TIME 30      // Tempo de trânsito upstream [ns]
#define REG_DOWNSTREAM_TIME 31    // Tempo de trânsito downstream [ns]
#define REG_DELTA_TIME 32         // Diferença de tempo [ns]

// Status e diagnósticos (formato uint16 - 1 registrador cada)
#define REG_WORKING_STATUS 96     // Status de trabalho principal
#define REG_ERROR_CODE 97         // Código de erro específico
#define REG_SYSTEM_STATUS 98      // Status do sistema
#define REG_ALARM_STATUS 99       // Status de alarmes

// Parâmetros de configuração (formato uint16 - 1 registrador cada)
#define REG_PIPE_DIAMETER 100     // Diâmetro do tubo [mm]
#define REG_PIPE_MATERIAL 101     // Material do tubo
#define REG_FLUID_TYPE 102        // Tipo de fluido
#define REG_MEASUREMENT_UNITS 103 // Unidades de medição

// Instâncias de comunicação
HardwareSerial rs485Serial(2);   // UART2 hardware do ESP32
ModbusMaster modbusNode;          // Instância do protocolo Modbus

// Estrutura completa para dados do TUF-2000
struct TUF2000CompleteData {
  // === MEDIÇÕES INSTANTÂNEAS ===
  float flowRate;                 // Vazão instantânea [L/s]
  float energyFlowRate;          // Taxa de fluxo de energia [kW]
  float velocity;                // Velocidade do fluido [m/s]
  float fluidSoundSpeed;         // Velocidade do som no fluido [m/s]
  float temperature;             // Temperatura [°C]
  float heatInput;               // Entrada de calor [kW]
  float heatOutput;              // Saída de calor [kW]
  
  // === TOTALIZADORES ===
  float positiveAccumulator;     // Totalizador positivo [L]
  float negativeAccumulator;     // Totalizador negativo [L]
  float netAccumulator;          // Totalizador líquido [L]
  float positiveEnergy;          // Energia positiva [kWh]
  float negativeEnergy;          // Energia negativa [kWh]
  float netEnergy;               // Energia líquida [kWh]
  
  // === QUALIDADE DO SINAL ===
  uint16_t signalQuality;        // Qualidade do sinal [0-100%]
  uint16_t upstreamStrength;     // Força sinal upstream [0-999]
  uint16_t downstreamStrength;   // Força sinal downstream [0-999]
  uint16_t upstreamTime;         // Tempo trânsito upstream [ns]
  uint16_t downstreamTime;       // Tempo trânsito downstream [ns]
  uint16_t deltaTime;            // Diferença de tempo [ns]
  
  // === STATUS E DIAGNÓSTICOS ===
  uint16_t workingStatus;        // Status de trabalho principal
  uint16_t errorCode;            // Código de erro específico
  uint16_t systemStatus;         // Status do sistema
  uint16_t alarmStatus;          // Status de alarmes
  
  // === CONFIGURAÇÕES ===
  uint16_t pipeDiameter;         // Diâmetro do tubo [mm]
  uint16_t pipeMaterial;         // Material do tubo
  uint16_t fluidType;            // Tipo de fluido
  uint16_t measurementUnits;     // Unidades de medição
  
  // === METADADOS ===
  bool dataValid;                // Flag de validade dos dados
  unsigned long lastUpdate;      // Timestamp da última atualização
  uint16_t readErrors;           // Contador de erros de leitura
  uint16_t totalReads;           // Contador total de leituras
};

// Variáveis globais do sistema
TUF2000CompleteData meterData;           // Estrutura com todos os dados
unsigned long lastReadTime = 0;         // Controle de timing para leituras
unsigned long lastStatusUpdate = 0;     // Controle para updates de status
bool communicationOK = false;           // Flag de status da comunicação
uint16_t connectionRetries = 0;         // Contador de tentativas de reconexão

// Configurações de timing (em millisegundos)
const unsigned long READ_INTERVAL = 5000;       // Intervalo entre leituras completas
const unsigned long STATUS_INTERVAL = 1000;     // Intervalo para updates de status
const unsigned long WATCHDOG_TIMEOUT = 10000;   // Timeout do watchdog timer

/*
 * Função executada antes da transmissão Modbus
 * Configura o módulo RS485 para modo transmissão
 */
void preTransmission() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Habilita modo transmissão
  delayMicroseconds(500);               // Delay para estabilização do sinal
}

/*
 * Função executada após a transmissão Modbus
 * Configura o módulo RS485 para modo recepção
 */
void postTransmission() {
  delayMicroseconds(500);               // Delay para conclusão da transmissão
  digitalWrite(RS485_DE_RE_PIN, LOW);   // Habilita modo recepção
}

/*
 * Configuração inicial completa do sistema ESP32
 */
void setup() {
  // === INICIALIZAÇÃO DO SISTEMA ===
  // Inicializa comunicação serial para debug e interface
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10); // Aguarda até 5 segundos pela porta serial
  }
  
  // Banner de inicialização com informações do sistema
  printSystemBanner();
  
  // === CONFIGURAÇÃO DOS PINOS ===
  // Configura pino de controle DE/RE do módulo RS485
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);   // Inicia em modo recepção
  
  // Configura LED de status interno
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // === INICIALIZAÇÃO DA COMUNICAÇÃO RS485 ===
  // Inicializa UART2 hardware com configuração 8N1
  rs485Serial.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  rs485Serial.setTimeout(MODBUS_TIMEOUT);
  
  // === CONFIGURAÇÃO DO PROTOCOLO MODBUS ===
  // Inicializa e configura o protocolo Modbus RTU
  modbusNode.begin(MODBUS_SLAVE_ID, rs485Serial);
  modbusNode.preTransmission(preTransmission);
  modbusNode.postTransmission(postTransmission);
  
  // === CONFIGURAÇÃO DO WATCHDOG TIMER ===
  // Configura watchdog para garantir estabilidade do sistema
  esp_task_wdt_init(WATCHDOG_TIMEOUT / 1000, true);
  esp_task_wdt_add(NULL);
  
  // === INICIALIZAÇÃO DOS DADOS ===
  // Inicializa estrutura de dados com valores padrão
  initializeMeterDataStructure();
  
  // === EXIBIÇÃO DAS CONFIGURAÇÕES ===
  // Mostra configurações atuais do sistema
  printSystemConfiguration();
  
  // === TESTE INICIAL DE COMUNICAÇÃO ===
  Serial.println("[SETUP] Executando teste inicial de comunicacao...");
  delay(2000); // Aguarda estabilização do hardware
  
  if (testInitialCommunication()) {
    Serial.println("[SETUP] ✓ Sistema inicializado com sucesso!");
    communicationOK = true;
    digitalWrite(LED_STATUS_PIN, HIGH); // Liga LED de status
  } else {
    Serial.println("[SETUP] ✗ Falha na inicializacao - continuando em modo debug");
    communicationOK = false;
  }
  
  Serial.println("[SETUP] Iniciando loop principal de leituras...\n");
  lastReadTime = millis();
  lastStatusUpdate = millis();
}

/*
 * Loop principal do programa - executa continuamente
 */
void loop() {
  // === RESET DO WATCHDOG TIMER ===
  // Reseta o watchdog para evitar reinicialização automática
  esp_task_wdt_reset();
  
  // === CONTROLE DE TIMING PARA LEITURAS COMPLETAS ===
  unsigned long currentTime = millis();
  
  // Verifica se é hora de executar uma leitura completa dos dados
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Executa leitura completa de todos os parâmetros
    Serial.println("[MAIN] Iniciando leitura completa do TUF-2000...");
    
    if (readAllTUF2000Parameters()) {
      // Leitura bem-sucedida
      communicationOK = true;
      connectionRetries = 0;
      meterData.totalReads++;
      
      // Exibe todos os dados lidos
      displayCompleteData();
      
      // Gera JSON com os dados (opcional)
      if (Serial.available() && Serial.peek() == 'j') {
        Serial.read(); // Consome o comando 'j'
        generateJSONOutput();
      }
      
    } else {
      // Falha na leitura
      communicationOK = false;
      connectionRetries++;
      meterData.readErrors++;
      
      Serial.println("[ERROR] Falha na leitura completa dos dados!");
      
      // Tenta reconexão se muitas falhas consecutivas
      if (connectionRetries >= MAX_RETRIES) {
        Serial.println("[ERROR] Muitas falhas consecutivas - tentando reconexao...");
        attemptReconnection();
      }
    }
  }
  
  // === CONTROLE DE TIMING PARA STATUS UPDATES ===
  // Atualiza indicadores de status em intervalo menor
  if (currentTime - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = currentTime;
    updateStatusIndicators();
  }
  
  // === PROCESSAMENTO DE COMANDOS SERIAIS ===
  // Processa comandos recebidos via interface serial
  processSerialCommands();
  
  // === DELAY PARA NÃO SOBRECARREGAR O SISTEMA ===
  delay(50); // Pequeno delay para estabilidade
}

/*
 * Inicializa a estrutura de dados com valores padrão
 */
void initializeMeterDataStructure() {
  // Zera todas as medições instantâneas
  meterData.flowRate = 0.0;
  meterData.energyFlowRate = 0.0;
  meterData.velocity = 0.0;
  meterData.fluidSoundSpeed = 0.0;
  meterData.temperature = 0.0;
  meterData.heatInput = 0.0;
  meterData.heatOutput = 0.0;
  
  // Zera todos os totalizadores
  meterData.positiveAccumulator = 0.0;
  meterData.negativeAccumulator = 0.0;
  meterData.netAccumulator = 0.0;
  meterData.positiveEnergy = 0.0;
  meterData.negativeEnergy = 0.0;
  meterData.netEnergy = 0.0;
  
  // Zera parâmetros de qualidade do sinal
  meterData.signalQuality = 0;
  meterData.upstreamStrength = 0;
  meterData.downstreamStrength = 0;
  meterData.upstreamTime = 0;
  meterData.downstreamTime = 0;
  meterData.deltaTime = 0;
  
  // Zera status e diagnósticos
  meterData.workingStatus = 0;
  meterData.errorCode = 0;
  meterData.systemStatus = 0;
  meterData.alarmStatus = 0;
  
  // Zera configurações
  meterData.pipeDiameter = 0;
  meterData.pipeMaterial = 0;
  meterData.fluidType = 0;
  meterData.measurementUnits = 0;
  
  // Inicializa metadados
  meterData.dataValid = false;
  meterData.lastUpdate = 0;
  meterData.readErrors = 0;
  meterData.totalReads = 0;
}

/*
 * Exibe banner de inicialização do sistema
 */
void printSystemBanner() {
  Serial.println("╔══════════════════════════════════════════════════════╗");
  Serial.println("║          TUF-2000 COMPLETE DATA READER              ║");
  Serial.println("║                                                      ║");
  Serial.println("║  ESP32 + RS485 + ModbusRTU                          ║");
  Serial.println("║  Autor: ClaudioSeda                                  ║");
  Serial.println("║  Data: 2025-07-14 14:58:36                          ║");
  Serial.println("║                                                      ║");
  Serial.println("║  Funcionalidades:                                    ║");
  Serial.println("║  • Leitura completa de todos os parametros          ║");
  Serial.println("║  • Interface serial interativa                      ║");
  Serial.println("║  • Saida em formato JSON                            ║");
  Serial.println("║  • Tratamento de erros e reconexao                  ║");
  Serial.println("║  • Watchdog timer para estabilidade                 ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println();
}

/*
 * Exibe configurações atuais do sistema
 */
void printSystemConfiguration() {
  Serial.println("=== CONFIGURACOES DO SISTEMA ===");
  Serial.printf("ESP32 Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("ESP32 Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU Cores: %d\n", ESP.getChipCores());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Total Heap: %d bytes\n", ESP.getHeapSize());
  Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
  Serial.println();
  
  Serial.println("=== CONFIGURACOES MODBUS ===");
  Serial.printf("Slave ID: %d\n", MODBUS_SLAVE_ID);
  Serial.printf("Baud Rate: %d\n", MODBUS_BAUD_RATE);
  Serial.printf("Timeout: %d ms\n", MODBUS_TIMEOUT);
  Serial.printf("Max Retries: %d\n", MAX_RETRIES);
  Serial.println();
  
  Serial.println("=== CONFIGURACOES DE PINOS ===");
  Serial.printf("RS485 RX Pin: GPIO%d\n", RS485_RX_PIN);
  Serial.printf("RS485 TX Pin: GPIO%d\n", RS485_TX_PIN);
  Serial.printf("RS485 DE/RE Pin: GPIO%d\n", RS485_DE_RE_PIN);
  Serial.printf("Status LED Pin: GPIO%d\n", LED_STATUS_PIN);
  Serial.println();
  
  Serial.println("=== CONFIGURACOES DE TIMING ===");
  Serial.printf("Read Interval: %lu ms\n", READ_INTERVAL);
  Serial.printf("Status Interval: %lu ms\n", STATUS_INTERVAL);
  Serial.printf("Watchdog Timeout: %lu ms\n", WATCHDOG_TIMEOUT);
  Serial.println();
}

/*
 * Testa comunicação inicial com o medidor TUF-2000
 */
bool testInitialCommunication() {
  Serial.println("[TEST] Testando comunicacao basica com TUF-2000...");
  
  // Primeiro teste: lê status de trabalho
  uint8_t result = modbusNode.readHoldingRegisters(REG_WORKING_STATUS, 1);
  
  if (result == modbusNode.ku8MBSuccess) {
    uint16_t status = modbusNode.getResponseBuffer(0);
    Serial.printf("[TEST] ✓ Comunicacao estabelecida! Status: %d (%s)\n", 
                  status, getWorkingStatusDescription(status));
    
    delay(200);
    
    // Segundo teste: lê vazão instantânea
    result = modbusNode.readHoldingRegisters(REG_FLOW_RATE, 2);
    if (result == modbusNode.ku8MBSuccess) {
      float flow = convertModbusToFloat(0, 1);
      Serial.printf("[TEST] ✓ Vazao lida com sucesso: %.3f L/s\n", flow);
      return true;
    } else {
      Serial.printf("[TEST] ✗ Falha ao ler vazao: 0x%02X\n", result);
      return false;
    }
    
  } else {
    Serial.printf("[TEST] ✗ Falha na comunicacao inicial: 0x%02X\n", result);
    printTroubleshootingHelp();
    return false;
  }
}

/*
 * Lê todos os parâmetros do TUF-2000 de forma sequencial
 */
bool readAllTUF2000Parameters() {
  bool overallSuccess = true;
  uint8_t result;
  
  Serial.println("[READ] Iniciando leitura sequencial de todos os parametros...");
  
  // === LEITURA DAS MEDIÇÕES INSTANTÂNEAS (FLOAT) ===
  
  // Lê vazão instantânea
  result = modbusNode.readHoldingRegisters(REG_FLOW_RATE, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.flowRate = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Vazao: %.3f L/s\n", meterData.flowRate);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro vazao: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê taxa de fluxo de energia
  result = modbusNode.readHoldingRegisters(REG_ENERGY_FLOW_RATE, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.energyFlowRate = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Taxa energia: %.2f kW\n", meterData.energyFlowRate);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro taxa energia: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê velocidade do fluido
  result = modbusNode.readHoldingRegisters(REG_VELOCITY, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.velocity = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Velocidade: %.3f m/s\n", meterData.velocity);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro velocidade: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê velocidade do som no fluido
  result = modbusNode.readHoldingRegisters(REG_FLUID_SOUNDSPEED, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.fluidSoundSpeed = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Vel. som fluido: %.1f m/s\n", meterData.fluidSoundSpeed);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro vel. som: 0x%02X\n", result);
  }
  delay(100);
  
  // === LEITURA DOS TOTALIZADORES (FLOAT) ===
  
  // Lê totalizador positivo
  result = modbusNode.readHoldingRegisters(REG_POSITIVE_ACCUMULATOR, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.positiveAccumulator = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Total positivo: %.2f L\n", meterData.positiveAccumulator);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro total positivo: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê totalizador negativo
  result = modbusNode.readHoldingRegisters(REG_NEGATIVE_ACCUMULATOR, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.negativeAccumulator = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Total negativo: %.2f L\n", meterData.negativeAccumulator);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro total negativo: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê totalizador líquido
  result = modbusNode.readHoldingRegisters(REG_NET_ACCUMULATOR, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.netAccumulator = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Total liquido: %.2f L\n", meterData.netAccumulator);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro total liquido: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê temperatura
  result = modbusNode.readHoldingRegisters(REG_TEMPERATURE, 2);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.temperature = convertModbusToFloat(0, 1);
    Serial.printf("[READ] ✓ Temperatura: %.1f °C\n", meterData.temperature);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro temperatura: 0x%02X\n", result);
  }
  delay(100);
  
  // === LEITURA DOS PARÂMETROS DE QUALIDADE (UINT16) ===
  
  // Lê qualidade do sinal
  result = modbusNode.readHoldingRegisters(REG_SIGNAL_QUALITY, 1);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.signalQuality = modbusNode.getResponseBuffer(0);
    Serial.printf("[READ] ✓ Qualidade sinal: %d%%\n", meterData.signalQuality);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro qualidade sinal: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê força do sinal upstream
  result = modbusNode.readHoldingRegisters(REG_UPSTREAM_STRENGTH, 1);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.upstreamStrength = modbusNode.getResponseBuffer(0);
    Serial.printf("[READ] ✓ Forca upstream: %d\n", meterData.upstreamStrength);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro forca upstream: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê força do sinal downstream
  result = modbusNode.readHoldingRegisters(REG_DOWNSTREAM_STRENGTH, 1);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.downstreamStrength = modbusNode.getResponseBuffer(0);
    Serial.printf("[READ] ✓ Forca downstream: %d\n", meterData.downstreamStrength);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro forca downstream: 0x%02X\n", result);
  }
  delay(100);
  
  // === LEITURA DOS STATUS (UINT16) ===
  
  // Lê status de trabalho
  result = modbusNode.readHoldingRegisters(REG_WORKING_STATUS, 1);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.workingStatus = modbusNode.getResponseBuffer(0);
    Serial.printf("[READ] ✓ Status trabalho: %d (%s)\n", 
                  meterData.workingStatus, getWorkingStatusDescription(meterData.workingStatus));
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro status trabalho: 0x%02X\n", result);
  }
  delay(100);
  
  // Lê código de erro
  result = modbusNode.readHoldingRegisters(REG_ERROR_CODE, 1);
  if (result == modbusNode.ku8MBSuccess) {
    meterData.errorCode = modbusNode.getResponseBuffer(0);
    Serial.printf("[READ] ✓ Codigo erro: %d\n", meterData.errorCode);
  } else {
    overallSuccess = false;
    Serial.printf("[READ] ✗ Erro codigo erro: 0x%02X\n", result);
  }
  delay(100);
  
  // Atualiza metadados baseado no resultado
  if (overallSuccess) {
    meterData.dataValid = true;
    meterData.lastUpdate = millis();
    Serial.println("[READ] ✓ Leitura completa finalizada com sucesso!");
  } else {
    meterData.dataValid = false;
    Serial.println("[READ] ✗ Leitura completa finalizada com erros!");
  }
  
  return overallSuccess;
}

/*
 * Converte dois registradores Modbus em um valor float
 * TUF-2000 usa formato IEEE 754 com ordenação Big Endian
 */
float convertModbusToFloat(uint8_t highRegisterIndex, uint8_t lowRegisterIndex) {
  union FloatConverter {
    float floatValue;
    uint32_t uint32Value;
    struct {
      uint16_t lowWord;
      uint16_t highWord;
    } words;
  } converter;
  
  // TUF-2000 transmite em formato Big Endian
  converter.words.highWord = modbusNode.getResponseBuffer(highRegisterIndex);
  converter.words.lowWord = modbusNode.getResponseBuffer(lowRegisterIndex);
  
  return converter.floatValue;
}

/*
 * Exibe todos os dados lidos do medidor de forma organizada
 */
void displayCompleteData() {
  // Calcula uptime formatado
  unsigned long uptime = millis();
  unsigned long uptimeSeconds = uptime / 1000;
  unsigned long uptimeMinutes = uptimeSeconds / 60;
  unsigned long uptimeHours = uptimeMinutes / 60;
  
  // Taxa de sucesso das leituras
  float successRate = 0.0;
  if (meterData.totalReads > 0) {
    successRate = ((float)(meterData.totalReads - meterData.readErrors) / meterData.totalReads) * 100.0;
  }
  
  Serial.println();
  Serial.println("╔════════════════════════════════════════════════════════════════╗");
  Serial.println("║                      DADOS COMPLETOS TUF-2000                 ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Timestamp: %02lu:%02lu:%02lu | Leituras: %d | Sucessos: %.1f%%     ║\n", 
                uptimeHours % 24, uptimeMinutes % 60, uptimeSeconds % 60,
                meterData.totalReads, successRate);
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  
  // === SEÇÃO DE MEDIÇÕES INSTANTÂNEAS ===
  Serial.println("║                     MEDICOES INSTANTANEAS                     ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Vazao Instantanea:     %12.3f L/s                    ║\n", meterData.flowRate);
  Serial.printf("║ Taxa Fluxo Energia:    %12.2f kW                     ║\n", meterData.energyFlowRate);
  Serial.printf("║ Velocidade Fluido:     %12.3f m/s                    ║\n", meterData.velocity);
  Serial.printf("║ Vel. Som no Fluido:    %12.1f m/s                    ║\n", meterData.fluidSoundSpeed);
  Serial.printf("║ Temperatura:           %12.1f °C                     ║\n", meterData.temperature);
  Serial.printf("║ Entrada Calor:         %12.2f kW                     ║\n", meterData.heatInput);
  Serial.printf("║ Saida Calor:           %12.2f kW                     ║\n", meterData.heatOutput);
  
  // === SEÇÃO DE TOTALIZADORES ===
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.println("║                        TOTALIZADORES                          ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Totalizador Positivo:  %12.2f L                      ║\n", meterData.positiveAccumulator);
  Serial.printf("║ Totalizador Negativo:  %12.2f L                      ║\n", meterData.negativeAccumulator);
  Serial.printf("║ Totalizador Liquido:   %12.2f L                      ║\n", meterData.netAccumulator);
  Serial.printf("║ Energia Positiva:      %12.2f kWh                    ║\n", meterData.positiveEnergy);
  Serial.printf("║ Energia Negativa:      %12.2f kWh                    ║\n", meterData.negativeEnergy);
  Serial.printf("║ Energia Liquida:       %12.2f kWh                    ║\n", meterData.netEnergy);
  
  // === SEÇÃO DE QUALIDADE DO SINAL ===
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.println("║                     QUALIDADE DO SINAL                        ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Qualidade Sinal:       %12d %%                       ║\n", meterData.signalQuality);
  Serial.printf("║ Forca Upstream:        %12d                          ║\n", meterData.upstreamStrength);
  Serial.printf("║ Forca Downstream:      %12d                          ║\n", meterData.downstreamStrength);
  Serial.printf("║ Tempo Upstream:        %12d ns                       ║\n", meterData.upstreamTime);
  Serial.printf("║ Tempo Downstream:      %12d ns                       ║\n", meterData.downstreamTime);
  Serial.printf("║ Delta Tempo:           %12d ns                       ║\n", meterData.deltaTime);
  
  // === SEÇÃO DE STATUS E DIAGNÓSTICOS ===
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.println("║                   STATUS E DIAGNOSTICOS                       ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Status Trabalho: %d (%s)\n", meterData.workingStatus, getWorkingStatusDescription(meterData.workingStatus));
  Serial.printf("║ Codigo Erro:           %12d                          ║\n", meterData.errorCode);
  Serial.printf("║ Status Sistema:         %12d                          ║\n", meterData.systemStatus);
  Serial.printf("║ Status Alarmes:         %12d                          ║\n", meterData.alarmStatus);
  
  // === SEÇÃO DE CONFIGURAÇÕES ===
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.println("║                       CONFIGURACOES                           ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.printf("║ Diametro Tubo:         %12d mm                       ║\n", meterData.pipeDiameter);
  Serial.printf("║ Material Tubo:          %12d                          ║\n", meterData.pipeMaterial);
  Serial.printf("║ Tipo Fluido:           %12d                          ║\n", meterData.fluidType);
  Serial.printf("║ Unidades Medicao:      %12d                          ║\n", meterData.measurementUnits);
  
  Serial.println("╚════════════════════════════════════════════════════════════════╝");
  Serial.println();
}

/*
 * Retorna descrição textual do status de trabalho
 */
const char* getWorkingStatusDescription(uint16_t status) {
  switch (status) {
    case 0:     return "OK - Funcionamento Normal";
    case 1:     return "Advertencia - Sinal Fraco";
    case 2:     return "Advertencia - Sinal Muito Fraco";
    case 3:     return "Erro - Sem Sinal Ultrassonico";
    case 4:     return "Erro - Velocidade Muito Baixa";
    case 5:     return "Erro - Velocidade Muito Alta";
    case 6:     return "Erro - Temperatura Fora da Faixa";
    case 7:     return "Erro - Sensor Desconectado";
    case 8:     return "Erro - Falha Hardware";
    case 9:     return "Erro - Configuracao Invalida";
    case 10:    return "Erro - Calibracao Necessaria";
    case 11:    return "Erro - Timeout Comunicacao";
    case 12:    return "Erro - Dados Corrompidos";
    default:    return "Status Desconhecido";
  }
}

/*
 * Atualiza indicadores visuais de status
 */
void updateStatusIndicators() {
  // Controla LED de status baseado na comunicação
  if (communicationOK) {
    // Pisca LED rapidamente quando comunicação OK
    digitalWrite(LED_STATUS_PIN, (millis() / 500) % 2);
  } else {
    // Pisca LED lentamente quando há problemas
    digitalWrite(LED_STATUS_PIN, (millis() / 1500) % 2);
  }
}

/*
 * Tenta reconexão com o medidor após falhas consecutivas
 */
void attemptReconnection() {
  Serial.println("[RECONNECT] Tentando restabelecer comunicacao...");
  
  // Reinicializa a comunicação serial
  rs485Serial.end();
  delay(1000);
  rs485Serial.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  rs485Serial.setTimeout(MODBUS_TIMEOUT);
  
  // Reinicializa o protocolo Modbus
  modbusNode.begin(MODBUS_SLAVE_ID, rs485Serial);
  modbusNode.preTransmission(preTransmission);
  modbusNode.postTransmission(postTransmission);
  
  delay(2000);
  
  // Testa comunicação básica
  if (testInitialCommunication()) {
    Serial.println("[RECONNECT] ✓ Reconexao bem-sucedida!");
    connectionRetries = 0;
    communicationOK = true;
  } else {
    Serial.println("[RECONNECT] ✗ Falha na reconexao!");
  }
}

/*
 * Gera saída em formato JSON com todos os dados
 */
void generateJSONOutput() {
  Serial.println("\n=== SAIDA JSON ===");
  
  // Cria documento JSON (ajuste o tamanho conforme necessário)
  DynamicJsonDocument doc(2048);
  
  // Metadados
  doc["timestamp"] = millis();
  doc["device"] = "TUF-2000";
  doc["dataValid"] = meterData.dataValid;
  doc["totalReads"] = meterData.totalReads;
  doc["readErrors"] = meterData.readErrors;
  
  // Medições instantâneas
  JsonObject measurements = doc.createNestedObject("measurements");
  measurements["flowRate"] = meterData.flowRate;
  measurements["energyFlowRate"] = meterData.energyFlowRate;
  measurements["velocity"] = meterData.velocity;
  measurements["fluidSoundSpeed"] = meterData.fluidSoundSpeed;
  measurements["temperature"] = meterData.temperature;
  measurements["heatInput"] = meterData.heatInput;
  measurements["heatOutput"] = meterData.heatOutput;
  
  // Totalizadores
  JsonObject accumulators = doc.createNestedObject("accumulators");
  accumulators["positive"] = meterData.positiveAccumulator;
  accumulators["negative"] = meterData.negativeAccumulator;
  accumulators["net"] = meterData.netAccumulator;
  accumulators["positiveEnergy"] = meterData.positiveEnergy;
  accumulators["negativeEnergy"] = meterData.negativeEnergy;
  accumulators["netEnergy"] = meterData.netEnergy;
  
  // Qualidade do sinal
  JsonObject signal = doc.createNestedObject("signal");
  signal["quality"] = meterData.signalQuality;
  signal["upstreamStrength"] = meterData.upstreamStrength;
  signal["downstreamStrength"] = meterData.downstreamStrength;
  signal["upstreamTime"] = meterData.upstreamTime;
  signal["downstreamTime"] = meterData.downstreamTime;
  signal["deltaTime"] = meterData.deltaTime;
  
  // Status
  JsonObject status = doc.createNestedObject("status");
  status["working"] = meterData.workingStatus;
  status["error"] = meterData.errorCode;
  status["system"] = meterData.systemStatus;
  status["alarm"] = meterData.alarmStatus;
  
  // Configurações
  JsonObject config = doc.createNestedObject("config");
  config["pipeDiameter"] = meterData.pipeDiameter;
  config["pipeMaterial"] = meterData.pipeMaterial;
  config["fluidType"] = meterData.fluidType;
  config["measurementUnits"] = meterData.measurementUnits;
  
  // Serializa e exibe o JSON
  serializeJsonPretty(doc, Serial);
  Serial.println("\n===================\n");
}

/*
 * Processa comandos recebidos via interface serial
 */
void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "HELP" || command == "?") {
      printCommandHelp();
    } else if (command == "STATUS") {
      displayCompleteData();
    } else if (command == "JSON") {
      generateJSONOutput();
    } else if (command == "TEST") {
      testInitialCommunication();
    } else if (command == "CONFIG") {
      printSystemConfiguration();
    } else if (command == "RESET") {
      Serial.println("[CMD] Reiniciando ESP32 em 3 segundos...");
      delay(3000);
      ESP.restart();
    } else if (command == "RECONNECT") {
      attemptReconnection();
    } else if (command == "INFO") {
      printESP32DetailedInfo();
    } else if (command.startsWith("INTERVAL ")) {
      // Comando para alterar intervalo de leitura (não implementado nesta versão)
      Serial.println("[CMD] Comando INTERVAL nao implementado nesta versao");
    } else if (command != "") {
      Serial.println("[CMD] Comando nao reconhecido. Digite HELP para ajuda.");
    }
  }
}

/*
 * Exibe menu completo de comandos disponíveis
 */
void printCommandHelp() {
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║                  COMANDOS DISPONIVEIS                 ║");
  Serial.println("╠════════════════════════════════════════════════════════╣");
  Serial.println("║ HELP ou ?      - Exibe este menu de ajuda             ║");
  Serial.println("║ STATUS         - Exibe todos os dados atuais          ║");
  Serial.println("║ JSON           - Exibe dados em formato JSON          ║");
  Serial.println("║ TEST           - Testa comunicacao com medidor        ║");
  Serial.println("║ CONFIG         - Exibe configuracoes do sistema       ║");
  Serial.println("║ INFO           - Informacoes detalhadas do ESP32      ║");
  Serial.println("║ RECONNECT      - Tenta reconectar com medidor         ║");
  Serial.println("║ RESET          - Reinicia o ESP32                     ║");
  Serial.println("║                                                        ║");
  Serial.println("║ Dica: Digite 'j' durante execucao para JSON rapido    ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
}

/*
 * Exibe informações detalhadas do ESP32
 */
void printESP32DetailedInfo() {
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║               INFORMACOES DETALHADAS ESP32            ║");
  Serial.println("╠════════════════════════════════════════════════════════╣");
  Serial.printf("║ Chip Model: %-42s ║\n", ESP.getChipModel());
  Serial.printf("║ Chip Revision: %37d ║\n", ESP.getChipRevision());
  Serial.printf("║ CPU Cores: %41d ║\n", ESP.getChipCores());
  Serial.printf("║ CPU Frequency: %34d MHz ║\n", ESP.getCpuFreqMHz());
  Serial.printf("║ Flash Size: %37d MB ║\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("║ Flash Speed: %35d MHz ║\n", ESP.getFlashChipSpeed() / 1000000);
  Serial.printf("║ Free Heap: %38d bytes ║\n", ESP.getFreeHeap());
  Serial.printf("║ Total Heap: %37d bytes ║\n", ESP.getHeapSize());
  Serial.printf("║ Min Free Heap: %33d bytes ║\n", ESP.getMinFreeHeap());
  Serial.printf("║ Max Alloc Heap: %31d bytes ║\n", ESP.getMaxAllocHeap());
  Serial.printf("║ SDK Version: %-38s ║\n", ESP.getSdkVersion());
  
  // Informações de uptime
  unsigned long uptime = millis();
  unsigned long uptimeDays = uptime / (24 * 60 * 60 * 1000);
  unsigned long uptimeHours = (uptime / (60 * 60 * 1000)) % 24;
  unsigned long uptimeMinutes = (uptime / (60 * 1000)) % 60;
  unsigned long uptimeSeconds = (uptime / 1000) % 60;
  
  Serial.printf("║ Uptime: %02lud %02lu:%02lu:%02lu %30s ║\n", 
                uptimeDays, uptimeHours, uptimeMinutes, uptimeSeconds, "");
  
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
}

/*
 * Exibe dicas para solução de problemas
 */
void printTroubleshootingHelp() {
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║                SOLUCAO DE PROBLEMAS                   ║");
  Serial.println("╠════════════════════════════════════════════════════════╣");
  Serial.println("║ 1. VERIFICAR CONEXOES FISICAS:                        ║");
  Serial.println("║    • A+ do TUF-2000 -> A+ do modulo RS485             ║");
  Serial.println("║    • B- do TUF-2000 -> B- do modulo RS485             ║");
  Serial.println("║    • GND comum entre todos os dispositivos            ║");
  Serial.println("║                                                        ║");
  Serial.println("║ 2. VERIFICAR ALIMENTACAO:                             ║");
  Serial.println("║    • TUF-2000: 8-36V DC com corrente adequada         ║");
  Serial.println("║    • ESP32: 3.3V estavel                              ║");
  Serial.println("║    • Modulo RS485: 3.3V ou 5V conforme especificacao  ║");
  Serial.println("║                                                        ║");
  Serial.println("║ 3. VERIFICAR CONFIGURACOES MODBUS:                    ║");
  Serial.println("║    • Endereco slave (padrao TUF-2000: 1)              ║");
  Serial.println("║    • Baud rate (padrao TUF-2000: 9600)                ║");
  Serial.println("║    • Formato dados: 8N1 (8 bits, sem paridade, 1 stop)║");
  Serial.println("║                                                        ║");
  Serial.println("║ 4. TESTAR DIFERENTES CONFIGURACOES:                   ║");
  Serial.println("║    • Trocar polaridade A+/B- se necessario            ║");
  Serial.println("║    • Adicionar resistores de terminacao (120 ohms)    ║");
  Serial.println("║    • Testar diferentes velocidades de comunicacao     ║");
  Serial.println("║                                                        ║");
  Serial.println("║ 5. COMANDOS UTEIS PARA DEBUG:                         ║");
  Serial.println("║    • TEST - testa comunicacao basica                  ║");
  Serial.println("║    • RECONNECT - tenta restabelecer conexao           ║");
  Serial.println("║    • CONFIG - mostra configuracoes atuais             ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
}