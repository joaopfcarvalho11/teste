/*
LilyGo TTGO T-Display (ESP32) + DS3231 (RTC + Alarme em INT/SQW)

- Janela de medições: 05:00 <= hora < 23:00 (ou seja, 05:00 até 22:59)
- Intervalo entre medições: 5 minutos (300 s)
- Fora da janela (23:00..04:59): dormir até a próxima hora cheia
- Acorda pelo pino INT/SQW do DS3231 em GPIO32 via EXT0 (nível BAIXO)

Bibliotecas requeridas:
- Wire (padrão)
- DS3231.h (mantida por requisito; o código usa acesso I2C direto aos registradores para portabilidade)
- driver/rtc_io.h
- esp_sleep.h

Observação importante:
Para não "congelar o relógio" a cada wake de deep sleep, o ajuste do RTC
é feito apenas no primeiro boot (flag em RTC_DATA_ATTR).
*/
#include <Wire.h>
#include <SPI.h>      // <-- adicione isto
#include <RTClib.h>
#include <Arduino.h>
#include <Wire.h>
#include "driver/rtc_io.h"
#include "esp_sleep.h"

// ---------- Pinos e constantes de hardware ----------
#define SDA_PIN 21
#define SCL_PIN 22
#define WAKEUP_GPIO GPIO_NUM_33 // EXT0 wakeup pin (RTC IO)
#define WAKEUP_PIN 33 // para digitalRead
#define DS3231_ADDRESS 0x68

// ---------- Janela e intervalos ----------
static const uint8_t HORA_INICIO = 5; // janela ativa: 05:00...
static const uint8_t HORA_FIM_EXC = 23; // ...até 22:59 (23 é exclusivo)
static const uint32_t INTERVALO_MEDICAO_SEG = 5UL * 60UL; // 5 minutos

// ---------- Flags/estado mantidos em RTC (sobrevivem ao deep sleep) ----------
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR bool rtc_time_initialized = false; // Ajuste do RTC só no primeiro boot

// ---------- Estrutura simples para hora/data ----------
struct RtcTime {
  uint16_t year; // ex.: 2025
  uint8_t month; // 1..12
  uint8_t date; // 1..31
  uint8_t day; // 1..7 (1=Domingo no DS3231)
  uint8_t hour; // 0..23
  uint8_t minute; // 0..59
  uint8_t second; // 0..59
};

// ---------- Utilitários DS3231 (BCD e registradores) ----------
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }
static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }

static void ds3231_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t ds3231_read_reg(uint8_t reg) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static void ds3231_read_time(RtcTime &t) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)7);

  uint8_t s = Wire.read(); // 0x00
  uint8_t m = Wire.read(); // 0x01
  uint8_t h = Wire.read(); // 0x02
  uint8_t dow = Wire.read(); // 0x03
  uint8_t date = Wire.read();// 0x04
  uint8_t mon = Wire.read(); // 0x05
  uint8_t yr = Wire.read(); // 0x06

  t.second = bcd2dec(s & 0x7F);
  t.minute = bcd2dec(m & 0x7F);

  if (h & 0x40) {
    // Modo 12h (não esperamos isso, mas tratamos)
    bool pm = h & 0x20;
    uint8_t hour12 = bcd2dec(h & 0x1F);
    t.hour = (hour12 % 12) + (pm ? 12 : 0);
  } else {
    // Modo 24h
    t.hour = bcd2dec(h & 0x3F);
  }

  t.day = dow & 0x07; // 1..7 (1=Domingo)
  t.date = bcd2dec(date & 0x3F);
  t.month = bcd2dec(mon & 0x1F);
  t.year = 2000 + bcd2dec(yr);
}

// Zeller (1=Domingo, 2=Segunda, ..., 7=Sábado) — alinhado ao DS3231 (1=Domingo)
static uint8_t day_of_week(uint16_t y, uint8_t m, uint8_t d) {
  if (m < 3) { m += 12; y -= 1; }
  uint16_t K = y % 100;
  uint16_t J = y / 100;
  int h = (d + (13*(m + 1))/5 + K + (K/4) + (J/4) + 5*J) % 7; // 0=Sat,1=Sun...
  int dow = ((h + 6) % 7) + 1; // 1..7 com 1=Dom, 7=Sáb
  return (uint8_t)dow;
}

static uint8_t month_from_str(const char *mmm) {
  if (!mmm) return 1;
  if (!strncmp(mmm, "Jan", 3)) return 1;
  if (!strncmp(mmm, "Feb", 3)) return 2;
  if (!strncmp(mmm, "Mar", 3)) return 3;
  if (!strncmp(mmm, "Apr", 3)) return 4;
  if (!strncmp(mmm, "May", 3)) return 5;
  if (!strncmp(mmm, "Jun", 3)) return 6;
  if (!strncmp(mmm, "Jul", 3)) return 7;
  if (!strncmp(mmm, "Aug", 3)) return 8;
  if (!strncmp(mmm, "Sep", 3)) return 9;
  if (!strncmp(mmm, "Oct", 3)) return 10;
  if (!strncmp(mmm, "Nov", 3)) return 11;
  if (!strncmp(mmm, "Dec", 3)) return 12;
  return 1;
}

// Ajusta o relógio do DS3231 para a hora de compilação (feito somente no 1º boot)
static void set_rtc_from_compile_time() {
  // __DATE__ formato: "Mmm dd yyyy"
  // __TIME__ formato: "hh:mm:ss"
  char mmm[4]; mmm[3] = '\0';
  char dateStr[] = __DATE__;
  char timeStr[] = __TIME__;

  // Extrai mês
  mmm[0] = dateStr[0]; mmm[1] = dateStr[1]; mmm[2] = dateStr[2];
  uint8_t month = month_from_str(mmm);

  // Extrai dia (pode haver espaço à esquerda)
  uint8_t day = (uint8_t)atoi(&dateStr[4]);

  // Extrai ano
  uint16_t year = (uint16_t)atoi(&dateStr[7]);

  // Extrai hora:min:seg
  uint8_t hour = (uint8_t)atoi(&timeStr[0]);
  uint8_t minute = (uint8_t)atoi(&timeStr[3]);
  uint8_t second = (uint8_t)atoi(&timeStr[6]);

  uint8_t dow = day_of_week(year, month, day);

  // Escreve nos registradores de tempo (0x00..0x06)
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write((uint8_t)0x00);
  Wire.write(dec2bcd(second)); // 0x00
  Wire.write(dec2bcd(minute)); // 0x01
  Wire.write(dec2bcd(hour) & 0x3F); // 0x02 (24h, garante bit6=0)
  Wire.write(dec2bcd(dow)); // 0x03 (1..7, 1=Dom)
  Wire.write(dec2bcd(day)); // 0x04
  Wire.write(dec2bcd(month)); // 0x05 (ignora century)
  Wire.write(dec2bcd((uint8_t)(year - 2000))); // 0x06
  Wire.endTransmission();

  // Configura o Control Register (0x0E):
  // INTCN=1 (bit2) para usar INT/SQW como interrupção, SQW desabilitado por padrão
  uint8_t ctrl = ds3231_read_reg(0x0E);
  ctrl |= (1 << 2); // INTCN = 1
  ctrl &= ~((1 << 4) | (1 << 3)); // RS2=RS1=0 (frequência SQW não usada)
  ds3231_write_reg(0x0E, ctrl);

  // Limpa flags de alarme no Status Register (0x0F)
  uint8_t status = ds3231_read_reg(0x0F);
  status &= ~((1 << 1) | (1 << 0)); // limpa A2F e A1F
  ds3231_write_reg(0x0F, status);
}

// Limpa as flags de alarme e garante INT/SQW liberado (alto)
static void ds3231_clear_alarm_flags() {
  uint8_t status = ds3231_read_reg(0x0F);
  status &= ~((1 << 1) | (1 << 0)); // A2F/A1F = 0
  ds3231_write_reg(0x0F, status);
}

// Habilita a interrupção do Alarme 1 (A1IE) e INTCN
static void ds3231_enable_a1_interrupt() {
  uint8_t ctrl = ds3231_read_reg(0x0E);
  ctrl |= (1 << 2); // INTCN = 1 (usa INT/SQW como interrupção)
  ctrl |= (1 << 0); // A1IE = 1 (habilita Alarme 1)
  ctrl &= ~(1 << 1); // A2IE = 0 (não usado aqui)
  ds3231_write_reg(0x0E, ctrl);
}

// Programa o Alarme 1 (A1) para um HH:MM:SS específico do dia, ignorando dia/data.
// A1M1=0 (match seconds), A1M2=0 (match minutes), A1M3=0 (match hours), A1M4=1 (ignora day/date)
static void ds3231_set_alarm1_hms(uint8_t hour, uint8_t minute, uint8_t second) {
  // A1 segundos (0x07): A1M1=0 => bit7=0
  ds3231_write_reg(0x07, dec2bcd(second) & 0x7F);
  // A1 minutos (0x08): A1M2=0 => bit7=0
  ds3231_write_reg(0x08, dec2bcd(minute) & 0x7F);
  // A1 horas (0x09): A1M3=0 => bit7=0; 24h => bit6=0
  ds3231_write_reg(0x09, (dec2bcd(hour) & 0x3F));
  // A1 dia/data (0x0A): A1M4=1 => bit7=1 (ignora dia/data), DY/DT=0
  ds3231_write_reg(0x0A, 0x80); // 1000 0000b
}

// ---------- Funções solicitadas no enunciado ----------

// 1) Configura o A1 para "agora + seconds_from_now" (usar precisão de segundos)
void configure_ds3231_alarm_in_seconds(uint32_t seconds_from_now) {
  RtcTime now;
  ds3231_read_time(now);

  uint32_t total = (uint32_t)now.hour * 3600UL + (uint32_t)now.minute * 60UL + (uint32_t)now.second + seconds_from_now;
  uint8_t nextHour = (total / 3600UL) % 24;
  uint8_t nextMinute = (total % 3600UL) / 60UL;
  uint8_t nextSecond = total % 60UL;

  // Limpa possíveis flags pendentes e garante INT alto
  ds3231_clear_alarm_flags();

  // Programa A1 para HH:MM:SS calculado
  ds3231_set_alarm1_hms(nextHour, nextMinute, nextSecond);

  // Habilita interrupção do A1 em INT/SQW
  ds3231_enable_a1_interrupt();

  Serial.printf("Alarme configurado para %02u:%02u:%02u (em %lu s)\n",
                nextHour, nextMinute, nextSecond, (unsigned long)seconds_from_now);
}

// 2) Exibe motivo do wake-up (debug)
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Acordou por EXT0 (RTC DS3231)"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Acordou por EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Acordou por TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Acordou por TOUCHPAD"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Acordou por ULP"); break;
    case ESP_SLEEP_WAKEUP_GPIO: Serial.println("Acordou por GPIO"); break;
    default:
      Serial.printf("Acordou por reset/power-on (%d)\n", (int)wakeup_reason);
      break;
  }
}

// 3) Aguarda INT/SQW (GPIO32) ficar em nível ALTO, evitando wake imediato
bool wait_int_high_with_timeout(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeout_ms) {
    if (digitalRead(WAKEUP_PIN) == HIGH) return true;
    delay(2);
  }
  return (digitalRead(WAKEUP_PIN) == HIGH);
}

// ---------- Medição simulada ----------
void fazerMedicao() {
  // Simulação de leitura (substitua pelo seu sensor)
  int leitura = analogRead(34); // ex.: ADC no GPIO34 se quiser
  Serial.printf("[MEDICAO] Valor lido: %d\n", leitura);
}

// ---------- Auxiliares de janela/tempo ----------
static inline bool dentroJanela(uint8_t hour) {
  return (hour >= HORA_INICIO) && (hour < HORA_FIM_EXC);
}

static uint32_t segundos_ate_proxima_hora(uint8_t minute, uint8_t second) {
  // Próxima hora cheia = 3600 - (min*60 + sec); se já em HH:00:00, retorna 3600
  uint32_t passados = (uint32_t)minute * 60UL + (uint32_t)second;
  return 3600UL - passados;
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("======= Iniciando ESP32 + DS3231 =======");

  bootCount++;
  Serial.printf("Boot # %lu\n", (unsigned long)bootCount);
  print_wakeup_reason();

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // opcional (400 kHz)

  // Entrada com pull-up (INT/SQW é open-drain no DS3231; a maioria dos módulos já tem pull-up)
  pinMode(WAKEUP_PIN, INPUT_PULLUP);

  // Ajusta o RTC somente no primeiro boot (após power-on / upload)
  if (!rtc_time_initialized) {
    set_rtc_from_compile_time();
    rtc_time_initialized = true;
    Serial.println("RTC ajustado com a hora de compilacao (apenas no 1o boot).");
  } else {
    Serial.println("RTC ja estava ajustado (mantido entre ciclos de deep sleep).");
  }

  // Sempre limpe flags de alarme pendentes ao iniciar
  ds3231_clear_alarm_flags();

  // Garante que EXT0 vai considerar nível BAIXO como wake
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 0);
}

void loop() {
  // Lê hora atual
  RtcTime now;
  ds3231_read_time(now);

  Serial.printf("Agora: %02u:%02u:%02u %02u/%02u/%04u (DOW=%u)\n",
    now.hour, now.minute, now.second,
    now.date, now.month, now.year, now.day);

  if (dentroJanela(now.hour)) {
    // Dentro da janela (05:00..22:59) -> executar medição e programar +5 min
    Serial.println("Dentro da janela de medições (05:00..22:59). Executando medição...");
    fazerMedicao();

    // Programa alarme para 5 minutos à frente
    configure_ds3231_alarm_in_seconds(INTERVALO_MEDICAO_SEG);
  } else {
    // Fora da janela (23:00..04:59) -> dormir até a próxima hora cheia
    uint32_t s_ate_hora = segundos_ate_proxima_hora(now.minute, now.second);
    Serial.printf("Fora da janela (23:00..04:59). Dormindo ate a proxima hora cheia (em %lu s)...\n",
                  (unsigned long)s_ate_hora);

    configure_ds3231_alarm_in_seconds(s_ate_hora);
  }

  // Evita wake imediato: INT/SQW deve estar em nível alto antes de dormir
  ds3231_clear_alarm_flags(); // garantimos liberar INT se algo ficou pendente
  ds3231_enable_a1_interrupt(); // garante A1IE=1 e INTCN=1 (INT/SQW via alarme)
  if (!wait_int_high_with_timeout(500)) {
    Serial.println("Aviso: INT/SQW permaneceu baixo por muito tempo; tentando novamente limpar flags...");
    ds3231_clear_alarm_flags();
    (void)wait_int_high_with_timeout(500);
  }

  Serial.println("Indo para deep sleep. Aguardando alarme do DS3231 em GPIO32 (EXT0)...");
  Serial.flush();

  // Importante: liberar o GPIO32 do driver digital para o domínio RTC
  rtc_gpio_deinit(WAKEUP_GPIO);

  esp_deep_sleep_start();

  // Nunca chega aqui (após wake, recomeça no setup)
}
