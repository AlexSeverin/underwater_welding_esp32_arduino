/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/

#include "OneWire.h"
#include "DallasTemperature.h"
//#include <AccelStepper.h>
#include <analogWrite.h>

//UART для управления блоком питания (hardware uart2) 
#define RXD2 16
#define TXD2 17

// Датчик температуры воды DS18B20 (1-wire)
#define ONE_WIRE_BUS 15
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1 = { 0x28, 0x4E, 0xCF, 0xC4, 0x42, 0x20, 0x1, 0xE3 }; //ds18b20 address (НЕ МЕНЯТЬ!)

//твердотельные реле
#define Relay_pump 18
#define Relay_power 19

//концевые выключатели
#define Switch_near 25 //концевик ближний
#define Switch_far 26 //концевик дальний

//Датчик протечки нормальнозамкнутый (H2O "Контакт" New исп.2 (НЗ), Датчик протечки воды четырехпроводный)
#define Water_sensor 13 //Датчик протечки

//Красная кнопка
#define RED_button 12 //(кнопка)
#define RED_button_backlight 14 //подсветка кнопки (красная)

//Драйвер шагового двигателя
#define motor_step 2
#define motor_dir 0
#define motor_enable 4

//Параметры ШИМ
int PWM_FREQUENCY = 4800; //Частота импульсов STEP
//int DRIVER_DIV = 800; // Дискретность драйвера
//float ROTATION_SPEED = PWM_FREQUENCY / DRIVER_DIV; // При дискретности 800 имп. на оборот получаем 6 об/сек
//float LINEAR_SPEED = ROTATION_SPEED * 0.8; // При шаге винта 0.8 мм  (винт М5) получаем линейную скорость 4.8 мм/сек
int PWM_CHANNEL = 0;
int PWM_RESOUTION = 8; 
int GPIOPIN = 2; 
int dutyCycle = 51; //20% заполнение ШИМ сигнала

// ПАКЕТЫ ДАННЫХ ДЛЯ БЛОКА ПИТАНИЯ DPS8005 НЕ МЕНЯТЬ!!!
// Описание протокола обмена по запросу в интернете: DPS8005 CNC Communication  Protocol V1.2 .pdf
// Для вычисления контрольной суммы пакета (последние 2 байта):
// Вводим пакет в HEX на сайте  https://crccalc.com/
// Например, для установки 75 В на выходе: 010600001D4C
// Нажимаем CALK CRC-16
// Из таблички берем два байта CRC-16/MODBUS Result ЗАДОМ НАПЕРЕД!!!!
// Для 010600001D4C получаем CRC-16/MODBUS: 0x6F81, а мы берем 0x81, 0x6F
// В итоге пакет принимает вид: uint8_t set_75v[]  = {0x01, 0x06, 0x00, 0x00, 0x1D, 0x4C, 0x81, 0x6F};

uint8_t read_v_c[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x65, 0xCB}; //Read the displayed output voltage and current value.

uint8_t power_on[]  = {0x01, 0x06, 0x00, 0x09, 0x00, 0x01, 0x98, 0x08};
uint8_t power_off[]  = {0x01, 0x06, 0x00, 0x09, 0x00, 0x00, 0x59, 0xC8};
uint8_t set_15v[]  = {0x01, 0x06, 0x00, 0x00, 0x05, 0xDC, 0x8B, 0x03};
uint8_t set_50v[]  = {0x01, 0x06, 0x00, 0x00, 0x13, 0x88, 0x84, 0x9C};
uint8_t set_75v[]  = {0x01, 0x06, 0x00, 0x00, 0x1D, 0x4C, 0x81, 0x6F};
uint8_t set_79v[]  = {0x01, 0x06, 0x00, 0x00, 0x1E, 0xDC, 0x81, 0xF3};
uint8_t set_10ma[]  = {0x01, 0x06, 0x00, 0x01, 0x00, 0x0A, 0x58, 0x0D}; // Ток 10 мА для проверки наличия воды в аквариуме
uint8_t set_50ma[]  = {0x01, 0x06, 0x00, 0x01, 0x00, 0x32, 0x59, 0xDF}; // Ток 50 мА для проверки наличия воды в аквариуме
uint8_t set_1a[]  = {0x01, 0x06, 0x00, 0x01, 0x03, 0xE8, 0xD8, 0xB4};
uint8_t set_5000ma[]  = {0x01, 0x06, 0x00, 0x01, 0x13, 0x88, 0xD5, 0x5C};
uint8_t set_5100ma[]  = {0x01, 0x06, 0x00, 0x01, 0x13, 0xEC, 0xD4, 0xB7};


// Переменные статуса работы экспоната

// 0 - цикл горения дуги не выполняется (свободен)
// n - количество циклов зажигания дуги (максмум 5 за одно нажатие)
int arc_run = 0;
int arc_status = 0; // статус горения дуги - (0 - еще не зажглась, 1 - зажглась и горит)
int arc_cycles = 0; // количество зажиганий дуги за время работы экспоната




  
// Функция передачи пакета данных для блока питания DPS8005
uint8_t power_supply_message (uint8_t msg[], uint8_t msg_size, uint16_t time_out)
{
  uint16_t Time_out = time_out;  //максимальное ожижание успешной передачи в мс
  uint16_t trying = 10; //количество попыток передачи
  //Serial.write(msg, HEX);
  do {
    Serial2.flush(); // Очищаем буфер
    
    //Serial.write(msg, sizeof(msg));
    Serial2.write(msg, msg_size);
    while(Serial2.available() < 1 && time_out > 0)
    {
      delay(1);
      Time_out--;
    }
    
    Time_out = time_out;
    trying--;
  } while ((Serial2.available() < 1) && (trying > 0));
  
  //Serial.println(Serial2.available());
  
  if (Serial2.available() < 1) {
    digitalWrite(Relay_power, LOW); // отключить реле блока питания
    digitalWrite(Relay_pump, LOW); // отключить реле насоса
    digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
    Serial.println("POWER SUPPLY: FAIL");
    while(1) // Падаем в бесконечный цикл до перезагрузки
    {
      delay(1);
    }
    return 1; 
  }
  else {
     //Serial.println("POWER SUPPLY: OK");
    return 0;   
  } 
}


// Проверка возможных ошибок
void check_exhibit (void)
{
  
  if(digitalRead(Switch_near) == HIGH) // если ближний концевик сработал (ВЫГОРЕЛИ ЭЛЕКТРОДЫ)
  {
    delay(1);
    if(digitalRead(Switch_near) == HIGH) // если ближний концевик сработал (ВЫГОРЕЛИ ЭЛЕКТРОДЫ)(доп. проверка)
    {
      digitalWrite(motor_enable, HIGH); // motor off
      
      Serial.println("NEAR SWITCH IS ON! CHECK ELECTRODES!");
      //Serial.print('\n');
      digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
  
      digitalWrite(Relay_power, HIGH); // включить реле блока питания
      delay(10000);
      
      int error;
      error = power_supply_message(power_off, sizeof(power_off), 5000); // отключаем выход блока питания
      Serial2.flush(); // Очищаем буфер 
      
      //Съезжаем с концевика
      ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/5, PWM_RESOUTION);
      ledcWrite(PWM_CHANNEL, dutyCycle);
      digitalWrite(motor_dir, LOW); //отодвигаем
      digitalWrite(motor_enable, LOW); // motor on
    
      while (digitalRead(Switch_near) == HIGH) // ждем, пока каретка съедет с концевика
      {
        delay(1);
      }
      
      digitalWrite(motor_enable, HIGH); // motor off
      digitalWrite(Relay_power, LOW); // отключить реле блока питания
      digitalWrite(Relay_pump, LOW); // отключить реле насоса
      digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
      
      while(1) // Падаем в бесконечный цикл до перезагрузки
      {
        delay(1);
      }
    }  
  }

  if(digitalRead(Switch_far) == HIGH) // если дальний концевик сработал
  {
    delay(1);
    if(digitalRead(Switch_far) == HIGH) // если дальний концевик сработал (доп. проверка)
    {
      digitalWrite(motor_enable, HIGH); // motor off
      
      Serial.println("FAR SWITCH IS ON!");
      //Serial.print('\n');
      digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
  
      digitalWrite(Relay_power, HIGH); // включить реле блока питания
      delay(10000);
      
      int error;
      error = power_supply_message(power_off, sizeof(power_off), 5000); // отключаем выход блока питания
      Serial2.flush(); // Очищаем буфер 
      
      //Съезжаем с концевика
      ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/5, PWM_RESOUTION);
      ledcWrite(PWM_CHANNEL, dutyCycle);
      digitalWrite(motor_dir, HIGH); //пододвигаем
      digitalWrite(motor_enable, LOW); // motor on
      
      while (digitalRead(Switch_far) == HIGH) // ждем, пока каретка съедет с концевика
      {
        delay(1);
      }
      
      digitalWrite(motor_enable, HIGH); // motor off
    }
  }

  
  //Если датчик протечки сработал - отключаем силовую часть и гасим кнопку
  if(digitalRead(Water_sensor) == HIGH) //есть протечка
  {
    delay(1);
    if(digitalRead(Water_sensor) == HIGH) //есть протечка (доп. проверка)
    {
      digitalWrite(motor_enable, HIGH); // motor off
      digitalWrite(Relay_power, LOW); // отключить реле блока питания
      digitalWrite(Relay_pump, LOW); // отключить реле насоса
      digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
      Serial.println("WATER LEAK!!!");
      //Serial.print('\n');
      while(1) // Падаем в бесконечный цикл до перезагрузки
      {
        delay(1);
      }
    }
  }

  /*
  //Температура воды в аквариуме
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  int water_temp = sensors.getTempC(sensor1);
  Serial.print("Sensor 1(*C): ");
  Serial.print(water_temp); 
  Serial.print('\n'); ;
  
  if(water_temp > 50) // Температура воды более 50 градусов
  { 
    digitalWrite(Relay_power, LOW); // отключить реле блока питания
    digitalWrite(Relay_pump, LOW); // отключить реле насоса
    digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
    Serial.print("HIGH WATER TEMPERATURE");
    Serial.print('\n');
    while(1) // Падаем в бесконечный цикл до перезагрузки
    {
      delay(1);
    }
  }
  */

  // Если датчики не сработали - зажигаем кнопку - экспонат готов к работе
  if( digitalRead(Switch_far) == LOW && 
      digitalRead(Switch_near) == LOW &&
      digitalRead(Water_sensor) == LOW)
  {
    digitalWrite(RED_button_backlight, HIGH); // включить подсветку кнопки
  } 

}

void start_exhibit (void)
{
  // Запуск генерации сигнала STEP для драйвера шагового двигателя
  digitalWrite(motor_enable, HIGH); // motor off
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/5, PWM_RESOUTION);
  ledcAttachPin(GPIOPIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, dutyCycle);

  check_exhibit(); // проверка датчиков
  digitalWrite(RED_button_backlight, LOW); // отключаем подсветку кнопки пока не пройдем стартовый цикл

  
  digitalWrite(Relay_power, HIGH); // включить силовой блок питания
  Serial.println("RELAY POWER ON!");
  delay(10000);
    
  digitalWrite(motor_dir, LOW); //отодвигаем каретку
  digitalWrite(motor_enable, LOW); // motor on

  while (digitalRead(Switch_far) == LOW) // ждем, пока не сработает дальний концевик
  {
    delay(1);
  }
  
  digitalWrite(motor_enable, HIGH); // motor off
  Serial.println("FAR SWITCH IS ON! CHECK ELECTRODES!");
  
  delay(10000); // ждем 10 секунд для возможной замены электрода в крайнем дальнем положении каретки.
    
  digitalWrite(motor_dir, HIGH); //отодвигаем каретку
  digitalWrite(motor_enable, LOW); // motor on
  
  while (digitalRead(Switch_far) == HIGH) // ждем, пока каретка съедет с дальнего концевика
  {
    delay(1);
  }
  
  digitalWrite(motor_enable, HIGH); // motor off
  
  Serial.println("WATER LEVEL CHECK...");
  
  // Проверка наличия воды в аквариуме
  uint8_t error; // Статус ошибки блока питания (0 - ОК; 1 - ОШИБКА)
  
  error = power_supply_message(set_50ma, sizeof(set_50ma), 5000);
  Serial2.flush(); // Очищаем буфер  
    
  error = power_supply_message(set_75v, sizeof(set_75v), 5000);
  Serial2.flush(); // Очищаем буфер  

  error = power_supply_message(power_on, sizeof(power_on), 5000);
  Serial2.flush(); // Очищаем буфер  

  delay(5000);
  
  //проверяем ток и напряжение
  float voltage = 0;
  float current = 0;  
  
  error = power_supply_message(read_v_c, sizeof(read_v_c), 5000);
  while(Serial2.available()) {
    uint8_t byte_num = Serial2.available();
    // отсылаем то, что получили
    if (byte_num == 6) {
      uint8_t v = Serial2.read();
      uint8_t v_dec = Serial2.read();
      voltage = ((float)((v<<8) | v_dec))/100.0;
      Serial.print("Voltage, V: ");
      Serial.print(voltage);
      Serial.print('\n');
      
      uint8_t c = Serial2.read();
      uint8_t c_dec = Serial2.read();
      current = ((float)((c<<8) | c_dec));
      Serial.print("Current, mA: ");
      Serial.print(current);
      Serial.print('\n');
    }
    else Serial2.read();
  }
  Serial2.flush(); // Очищаем буфер

  // Если ток холостого хода составляет более 20 мА - вода присутствует
  if (current > 20)
  {
    Serial.println("WATER LEVEL OK!");
    
    //Первичная настройка блока питания
    uint8_t error; // Статус ошибки блока питания (0 - ОК; 1 - ОШИБКА)
    
    error = power_supply_message(power_off, sizeof(power_off), 5000); // отключаем выход блока питания
    Serial2.flush(); // Очищаем буфер  
    
    error = power_supply_message(set_5000ma, sizeof(set_5000ma), 5000); // устанавливаем выходной ток
    Serial2.flush(); // Очищаем буфер      Serial2.flush(); // Очищаем буфер  
      
    error = power_supply_message(set_75v, sizeof(set_75v), 5000); // устанавливаем максимальное напряжение
    Serial2.flush(); // Очищаем буфер  
  
    digitalWrite(Relay_pump, HIGH); // Включаем насос фильтрующей системы
    digitalWrite(RED_button_backlight, HIGH); // включить подсветку кнопки
    
    Serial.println("The exhibit is ready to work");  // Экспонат готов к работе
    //delay(5000);
  }
  else // Если нет - нужно долить воды. Отлючаем экспонат
  {
    digitalWrite(motor_enable, HIGH); // motor off
    digitalWrite(Relay_power, LOW); // отключить реле блока питания
    digitalWrite(Relay_pump, LOW); // отключить реле насоса
    digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
    
    Serial.println("ADD MORE WATER!!!");
    
    while(1) // Падаем в бесконечный цикл до перезагрузки
    {
      delay(1);
    }
  }
}





void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  Serial.println("START_CODE");

  // Драйвер шагового двигателя
//  pinMode(ledPin, OUTPUT);    // ledPin это сигнал STEP для драйвера шагового двигателя
  //pinMode(motor_step, OUTPUT);
  pinMode(motor_dir, OUTPUT);
  pinMode(motor_enable, OUTPUT);
  digitalWrite(motor_enable, HIGH); // motor off
  
  //твердотельные реле
  pinMode(Relay_pump, OUTPUT); // реле насоса
  digitalWrite(Relay_pump, LOW); // отключить реле насоса
  pinMode(Relay_power, OUTPUT); // реле блока питания
  digitalWrite(Relay_power, LOW); // отключить реле блока питания
  
  // Кнопка
  pinMode(RED_button, INPUT_PULLUP); //вход кнопки
  pinMode(RED_button_backlight, OUTPUT); // подсветка кнопки
  digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
  
  // Концевые выключатели
  pinMode(Switch_near, INPUT_PULLUP); //концевик ближний
  pinMode(Switch_far, INPUT_PULLUP); //концевик дальний  
  
  // датчик протечки
  pinMode(Water_sensor, INPUT_PULLUP); //датчик протечки

  //разрешение работы датчика температуры воды DS18b20
  sensors.begin(); //1-wire

  // Запуск экспоната
  start_exhibit();
}



// ОСНОВНОЙ ЦИКЛ
void loop() 
{
  // Проверка возможных ошибок
  check_exhibit();
  
  //digitalWrite(motor_enable, HIGH); // motor off
  uint8_t gap = 0; //мертвый ход

  if(digitalRead(RED_button) == LOW){ //пользователь нажал кнопку
    arc_run = 5;

    
    //ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/2, PWM_RESOUTION);
    //delay(10);
    //ledcWrite(PWM_CHANNEL, dutyCycle);
   //Serial.print("GO OUT");
   // Serial.print('\n');
    //digitalWrite(motor_dir, LOW); //отодвигаем если кнопка нажата 
   // digitalWrite(motor_enable, LOW); // motor on
  }

  if(arc_run > 0)
  {
    //проверяем ток и напряжение
    float voltage = 0;
    float current = 0;
    
    uint8_t error;
    error = power_supply_message(read_v_c, sizeof(read_v_c), 10000);
    while(Serial2.available()) {
      uint8_t byte_num = Serial2.available();
      // отсылаем то, что получили
      if (byte_num == 6) {
        uint8_t v = Serial2.read();
        uint8_t v_dec = Serial2.read();
        voltage = ((float)((v<<8) | v_dec))/100.0;
        Serial.print("Voltage, V: ");
        Serial.print(voltage);
        Serial.print('\n');
        
        uint8_t c = Serial2.read();
        uint8_t c_dec = Serial2.read();
        current = ((float)((c<<8) | c_dec));
        Serial.print("Current, mA: ");
        Serial.print(current);
        Serial.print('\n');
      }
      else Serial2.read();
    }
    Serial2.flush(); // Очищаем буфер

    //Проверка уровня жидкости по току электролиза
    if (current < 20 && voltage > 70.0) {
      digitalWrite(motor_enable, HIGH); // motor off
      digitalWrite(Relay_power, LOW); // отключить реле блока питания
      digitalWrite(Relay_pump, LOW); // отключить реле насоса
      digitalWrite(RED_button_backlight, LOW); // отключить подсветку кнопки
      
      Serial.println("ADD MORE WATER!");
      
      while(1) // Падаем в бесконечный цикл до перезагрузки
      {
        delay(1);
      }
    }
  
    // Проверка напряжения и тока
    if (voltage < 30.0 && voltage > 1.0 && current >= 20) 
    { 
      if (arc_status != 1) //если предыдущий цикл горения дуги не завершился
      {
        arc_status = 1; // изменить текущий статус
      }
       
      gap++; //мертвый ход поршня
      
      //if (gap < 2){
        //быстро отъезжаем, чтобы устранить мертвый ход
        ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/10, PWM_RESOUTION);
        ledcWrite(PWM_CHANNEL, dutyCycle);
        digitalWrite(motor_dir, LOW); //отодвигаем
        digitalWrite(motor_enable, LOW); // motor on
     /* } else {
        //медленно отъезжаем
        ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/30, PWM_RESOUTION);
        ledcWrite(PWM_CHANNEL, dutyCycle);
        digitalWrite(motor_dir, LOW); //отодвигаем
        digitalWrite(motor_enable, LOW); // motor on
      }*/
    }
    else  if (voltage >= 30.0 && current >= 4800)  //поддержание горения дуги
    {
      if (arc_status != 1) //если предыдущий цикл горения дуги не завершился
      {
        arc_status = 1; // изменить текущий статус
      }
      
      // очень медленно отъезжаем и разрываем дугу
      ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/200, PWM_RESOUTION);
      ledcWrite(PWM_CHANNEL, dutyCycle);
      digitalWrite(motor_dir, LOW); //отодвигаем
      digitalWrite(motor_enable, LOW); // motor on
    }
    else if (voltage <= 1.0) { //Если выход блока питания отключился
      digitalWrite(motor_enable, HIGH); // motor off
      
      //Включаем выход блока питания
      error = power_supply_message(power_on, sizeof(power_on), 10000);
      Serial2.flush(); // Очищаем буфер
    }
    else if (voltage >= 30.0) // Если электроды еще не сдвинуты до короткого замыкания
    {
      if (arc_status != 0 && current < 4000) //если предыдущий цикл горения дуги завершился
      {
        arc_run--; // уменьшить счетчик запрашиваемых циклов горения дуги
        
        arc_cycles++; // увеличить счетчик общего количества циклов горения дуги
        Serial.print("Total number of arcs: ");
        Serial.print(arc_cycles);
        Serial.print('\n');
        
        arc_status = 0; // изменить текущий статус
      }
       
      gap = 0;
      ledcSetup(PWM_CHANNEL, PWM_FREQUENCY/5, PWM_RESOUTION);
      ledcWrite(PWM_CHANNEL, dutyCycle);
      digitalWrite(motor_dir, HIGH); //пододвигаем
      digitalWrite(motor_enable, LOW); // motor on
    }    
   }

   
   else //Если нет запроса на зажигание дуги - отключаем выход блока питания
   {
    digitalWrite(motor_enable, HIGH); // motor off
    uint8_t error; // Статус ошибки блока питания
    error = power_supply_message(power_off, sizeof(power_off), 10000); // отключаем выход блока питания
    Serial2.flush(); // Очищаем буфер
    Serial.println("WAIT");
   }

  Serial.flush(); // Очищаем буфер

  
  //Stepper motor control
  
  /*
  unsigned long currentMillis = micros();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
   // digitalWrite(ledPin, ledState);
  }
  */

}
