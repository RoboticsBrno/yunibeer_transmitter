#include "../avrlib/sync_usart.hpp"
#include "../avrlib/usart1.hpp"
#include "../avrlib/bootseq.hpp"
#include "../avrlib/format.hpp"
using namespace avrlib;

int8_t get_pot(int index)
{
	ADMUX = (1<<ADLAR)|(index + 4);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC))
	{
	}
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC))
	{
	}

	int8_t res = ADCH - 128;

	if (res > 20)
		res -= 20;
	else if (res < -20)
		res += 20;
	else
		res = 0;

	return -res;
}

uint8_t get_buttons()
{
	return PINB & 0xfe;
}

int main()
{
	sync_usart<usart1, bootseq> rs232(38400);

	for (int j = 0; j < 10000; ++j)
	{
		asm __volatile__ ("nop");
	}

	send(rs232, "'1' -- text, '2' -- binary");

	DDRB = 0;
	PORTB = 0xff;
	
	ADMUX = (1<<ADLAR)|7;
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
					send_int(rs232, get_pot(i), 5);
				send_int(rs232, get_buttons(), 5);
				send(rs232, "\r\n");
				break;
			case 2:
				rs232.write(-128);
				rs232.write(get_pot(0));
				rs232.write(-get_pot(2));
				rs232.write(get_pot(1));
				rs232.write(get_pot(3));
				rs232.write(get_buttons());
				break;
			}
		}
	}
}
