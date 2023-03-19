#include <avr/io.h>   //definicje rejestrów wejścia/wyjścia i ich bitów dla mikrokontrolera AVR użytego w kodzie.
#include <util/delay.h> //plik nagłówkowy zawierający definicje funkcji, które implementują opóźnienia czasowe w mikrosekundach
#include <string.h> // plik nagłówkowy, który udostępnia funkcje do manipulowania tablicami znaków lub ciągów znaków

#define F_CPU 16000000UL // Częstotliwość zegara mikrokontrolera
#define ENA PD5 //zmienna służąca do kontrolowania kanału A w mostku L298N, co pozwala na kontrolować jednen z silników.
#define ENB PD6 // mienna służąca do kontrolowania kanału B w mostku L298N, co pozwala na kontrolować jednen z silników.
#define IN1 PD7 //zmienna służąca jako wejście do sterowania silnikami DC przez mostek L298N
#define IN2 PB0 //zmienna służąca jako wejście do sterowania silnikami DC przez mostek L298N
#define IN3 PB1 //zmienna służąca jako wejście do sterowania silnikami DC przez mostek L298N
#define IN4 PB3 //zmienna służąca jako wejście do sterowania silnikami DC przez mostek L298N
#define TRIG_PIN PC5 //impulsowo-napięciowy pin, mtórego funkcją jest wysłanie sygnału ultradźwiękowego do środowiska, aby określić odległość do obiektu
#define ECHO_PIN PC4 //pin używany do odczytu sygnału wysłanego przez ultradźwiękowy czujnik odległości
#define BAUD 9600  //stała oznaczająca szybkość transmisji danych w transmisji szeregowej / 9600 bitów na sekundę
#define LEFT PD2 //lewy czujnik linii
#define MIDDLE PD4 // środkowy czujnik linii
#define RIGHT PB2 // prawy czujnik linii


//Initialize the ports used in the code
void init_motors_ports(void);  //inicjacja portów używanych w kodzie
void forward(uint8_t speed); //funkcja jazdy prosto, speed - 8-bitowa zmienna określająca prędkość z ją powinny porzuszać się silniki DC
void backward(uint8_t speed); //funkcja jazdy w tył
void turn_right(uint8_t speed); //funkcja jazdy w lewo
void turn_left(uint8_t speed); //funkcja jazdy w prawo
void stop_car(void); //funkcja zatrzymania
void trigger_pulse(void); //funkcja generująca impuls na trig_pin, Czas trwania impulsu wynosi 10 mikrosekund - inicjuje pomiar odległości do obiektu
int read_echo(); // funkcja zwracająca informację o odległości do obiektu
void adc_init(); // inicjowanie portów ADC
uint8_t adc_read(uint8_t channel);//funkcja-odczytuje wartość analogową określonego kanału,channel określa kanał, którego wartość ma zostać odczytana
void line_follower(void); //funkcja line followera

void UART_init(); //inicjowanie portó komunikacji UART
void UART_transmit(uint8_t data);//funkcja służąca do przesyłania danych (8-bitowych) przez połączenie szeregowe
uint8_t UART_receive(void); //funkcja, która odbiera dane z protokołu komunikacyjnego UART i zwraca 8-bitową daną 


int main(void) 
{
       init_motors_ports();
       adc_init();
       DDRC |= (1 << TRIG_PIN); // Ustawienie pinu trigger jako wyjście
       DDRC &= ~(1 << ECHO_PIN); // Ustawienie pinu ECHO jako wejście
       DDRD|=(1<<PD1);//ustawienie pinu PD1 jako wyjście - tx transmisja danych przez UART
       UART_init();
       char data = UART_receive(); //odbieranie danych przez UART i zapisywanie ich w zmiennej data
      
        while (1)
             {
             if( data == 'f') 
            {
              trigger_pulse();
              int distance = read_echo();
                 if (distance < 20) {
                    stop_car();
                 } else {
                  forward(255) ;
            }
            }
             else if(data =='b')
            {
             trigger_pulse();
              int distance = read_echo();
                 if (distance < 20) {
                    stop_car();
                 } else {
                  backward(255) ;
            }
             }
             else if(data =='r')
            {
             trigger_pulse();
              int distance = read_echo();
                 if (distance < 20) {
                    stop_car();
                 } else {
                  turn_right(255) ;
            }
             }
            else if(data =='l')
            {
             trigger_pulse();
              int distance = read_echo();
                 if (distance < 20) {
                    stop_car();
                 } else {
                  turn_left(255) ;
            }
             }
            else if(data =='s')
            {
                 stop_car();
             
            }
            else if(data =='T')
            {
                  line_follower();

            
            }
             }
             
      return 0;
}


void init_motors_ports(void){
 DDRD |= (1 << ENA) | (1 << ENB) | (1 << IN1) | (1 << LEFT) | (1 << MIDDLE);
 DDRB |= (1 << IN2) | (1 << IN3) | (1 << IN4) | (1 << RIGHT);

 //USTAWIENIA DLA TIMERA
 TCCR1A |= (1 << WGM10) | (1 << WGM11);
 TCCR1B |= (1 << WGM12) | (1 << WGM13);
 TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1 << COM1B0);
 TCCR1B |= (1 << CS10) | (1 << CS11) | (1 << CS12);//preskaler 1024
 OCR1AH = 0x00; //rejest wysoki    wartości porównania dla Timera 1, porównanie z licznikiem Timera 1, po osiągnięciu wartości wyzwala się przerwanie
 OCR1AL = 0x00; //rejest niski      czestotliwość PWM - F_CPU/1024/(OCRr1AH*OCRIAL)

}
void forward(uint8_t speed) {
 PORTD |= (1 << ENA) | (1 << ENB) | (1 << IN1) ;
 PORTB |= (1 << IN4);
 OCR1AH = speed; 
 OCR1BL = speed;
}
void backward(uint8_t speed){
 PORTD |= (1 << ENA) | (1 << ENB);
 PORTB |= (1 << IN2) | (1 << IN3);
 
 OCR1AH = speed;
 OCR1BL = speed;
}
void turn_right(uint8_t speed){
 PORTD |= (1 << ENA) | (1 << ENB) | (1 << IN1);
 PORTB |= (1 << IN3);
 OCR1AL = speed;
 OCR1BL = 0x00;
}
void turn_left(uint8_t speed) {
 PORTD |= (1 << ENA) | (1 << ENB);
 PORTB |= (1 << IN2) | (1 << IN4);
 OCR1AL = 0x00;
 OCR1BL = speed;
}
void stop_car(void) {
 PORTD &= ~(1 << ENA) & ~(1 << ENB);
 PORTB &= ~(1 << IN1) & ~(1 << IN2) & ~(1 << IN3) & ~(1 << IN4);
 OCR1AL = 0x00;
 OCR1BL = 0x00;
}
void trigger_pulse(void) {
  // Wysłanie sygnału TRIGGER
  PORTC |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTC &= ~(1 << TRIG_PIN);
}

int read_echo() {
  // Oczekiwanie na sygnał ECHO
  while (!(PINC & (1 << ECHO_PIN))) {}

  // Pomiar czasu trwania sygnału ECHO
  int duration = 0;
  while (PINC & (1 << ECHO_PIN)) {
    duration++;
    _delay_us(1);
  }
  // Obliczenie odległości
  int distance = duration / 58;  // (duration / 2) / (340 m/s) / (100 cm/m) odległość dzielona przez prędkość dźwięku (340 m/s), a następnie podwojona aby uwzględnić powrót sygnału
  return distance;
}

void UART_init()
{
// Ustawienie baudrate
UBRR0H = 0;
UBRR0L = 103; //UART 9600 //UBRR = F_CPU / (16 * BAUD) - 1
UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00); // Ustaw format 8n1
UCSR0B |= (1<<RXEN0)|(1<<TXEN0); //Włącz odbiornik i nadajnik
}
void UART_transmit(uint8_t data) //Funkcja wysyłania danych
{
while(!(UCSR0A & (1<<UDRE0))); // czekaj aż transmiter(bufor) gotowy
UDR0 = data;
}
uint8_t UART_receive(void) //Funkcja odbierania danych
{
while(!(UCSR0A & (1<<RXC0))); // czekaj aż znak(bajt) dostępny
return UDR0;
}

void adc_init()
{
  ADMUX = (1<<REFS0); // AVCC with external capacitor at AREF pin
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Enable ADC and set prescaler to 128
}


uint8_t adc_read(uint8_t channel)  {
  if (channel > 7) {
    return 255; // Return an error value if channel is out of range
  }
  
  ADMUX = (ADMUX & 0xF8) | channel;
  if (!(ADCSRA & (1 << ADEN))) {
    adc_init(); // Ensure ADC is enabled before starting a conversion
  }
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return (ADC);
}

void line_follower(void) 
{
  
  uint8_t left_value = adc_read(LEFT);
  uint8_t middle_value = adc_read(MIDDLE);
  uint8_t right_value = adc_read(RIGHT);
    
   if (left_value > middle_value >right_value) {
    stop_car();
  } else if (left_value > middle_value <  right_value) {
    turn_left(255);
  } else if (left_value <middle_value >right_value) {
    forward(255);
  } else if (left_value < middle_value <right_value ) {
    turn_right(255);
  } 
}
 

    
