#include <OrbicraftBus.h>
#include <Servo.h> 
Servo servo;
Servo servo2;
#define PIN_RELAY 5
/*
  * Объявим переменную msg как тип данных Message
  * Message - представляет собой структуру, описывающую идентификаторы передаваемого сообщения
*/
Message msg;
int i=0;
 
/*
  * Объявим переменную bus как тип данных OrbicraftBus 
  * OrbicraftBus - представляет собой класс, описывающий взаимодействие Arduino и шины конструктора Orbicraft
*/
OrbicraftBus bus;
 
// Объявим переменную msgSize, в которую будет записываться передаваемое сообщение
int16_t msgSize = 0;
// Объявим номер пина для считывания показаний

 
void setup() {
  servo.attach(10);
  servo2.attach(11);
  Serial.begin(9600);
   pinMode(PIN_RELAY, OUTPUT); // Объявляем пин реле как выход
  digitalWrite(PIN_RELAY, HIGH); // задаем скорость обмена информацией по Serial1 !!! (проверить Serial2.begin(9600))
}
 
void loop() {
 
  msgSize = bus.takeMessage(msg); // пробуем прочитать сообщение с помощью метода takeMessage
 
  if (msgSize > 0){ //если сообщение есть
    switch (msg.id){//в зависимости от идентификатора сообщения выполняем те или иные действия
 
      // Рассмотрим случай с идентификатором 2
        case 0x02:{
        
        servo.write(0); //ставим вал под 0
        delay(1000);
        servo2.write(0); //ставим вал под 0
        delay(1000);//ждем 2 секунды
        servo.write(180); //ставим вал под 180
        delay(1000);
        servo2.write(180); //ставим вал под 180
        delay(1000);
        servo.write(180); //ставим вал под 180
        delay(1000);
        servo2.write(180); //ставим вал под 180
        delay(1000);
        servo.write(0);
        delay(1000);
        servo2.write(0);
        
        //ждем 2 секунды
        
      
        // передаем содержимое переменной data на БКУ
        break;
        
      }
      case 0x01:{
         
          digitalWrite(PIN_RELAY, LOW); // Включаем реле - посылаем низкий уровень сигнала
          delay(5000);
          
         digitalWrite(PIN_RELAY, HIGH);
         
        break;
      }
    }
  }
}
 
 
/*
 * Следующий блок кода необходимо всегда добавлять в конец программы
 * Функция вызывается автоматически и необходима для обработки сообщения
*/ 
void serialEvent2() {
  bus.serialEventProcess();
}
