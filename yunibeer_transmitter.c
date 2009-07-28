#include "../avrlib/sync_usart.hpp"
#include "../avrlib/usart1.hpp"
#include "../avrlib/bootseq.hpp"
#include "../avrlib/format.hpp"
using namespace avrlib;

int16_t get_pot(int index)
{
	static uint8_t const channels[] = { 4, 5, 6, 7 };
	static bool const invert[] = { true, false, false, false };

	ADMUX = (1<<ADLAR) | channels[index];
	ADCSRA |= (1<<ADSC);
	while ((ADCSRA & (1<<ADIF)) == 0)
	{
	}
	ADCSRA |= (1<<ADSC);
	while ((ADCSRA & (1<<ADIF)) == 0)
	{
	}

	uint16_t res = ADCL;
	res |= ADCH << 8;

	if (invert[index])
		res = -res;

	return int16_t(res - 0x8000);
}

uint8_t get_buttons()
{
	return PINB;
}

int main()
{
	sync_usart<usart1, bootseq> rs232(38400);

	DDRB = 0;
	PORTB = 0xff;
	PORTF = 0;
	
//	ADMUX = (1<<ADLAR)|7;
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	TCCR0 = (1<<CS02)|(1<<CS01)|(1<<CS00);

	int send_state = 0; // 0 -- silent, 1 -- text, 2 -- binary
	
	PORTC = (1<<7);
	DDRC = (1<<5)|(1<<7);

	for (;;)
	{
		if (!rs232.empty())
		{
			switch (rs232.read())
			{
			case '1':
				send_state = 1;
				break;
			case '2':
				send_state = 2;
				PORTC ^= (1<<5)|(1<<7);
				break;
			case '?':
				send_state = 0;
				send(rs232, "'1' -- text, '2' -- binary\r\n");
				break;
			default:
				send_state = 0;
			}
		}

		if (TIFR & (1<<TOV0))
		{
			TIFR = (1<<TOV0);

			switch (send_state)
			{
			case 1:
				for (int i = 0; i < 4; ++i)
					send_int(rs232, get_pot(i), 7);
				send_int(rs232, get_buttons(), 5);
				send(rs232, "\r\n");
				break;
			case 2:
				rs232.write(0x80);
				rs232.write(0x19);
				for (int i = 0; i < 4; ++i)
					send_bin(rs232, get_pot(i));
				send_bin(rs232, get_buttons());
				break;
			}
		}
	}
}
