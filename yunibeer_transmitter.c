#include "avrlib/async_usart.hpp"
#include "avrlib/usart1.hpp"
#include "avrlib/bootseq.hpp"
#include "avrlib/format.hpp"
#include "avrlib/command_parser.hpp"
#include "avrlib/eeprom.hpp"
#include "avrlib/stopwatch.hpp"

#include "avrlib/pin.hpp"
#include "avrlib/portc.hpp"
using namespace avrlib;

template <typename Port1, int Pin1, typename Port2, int Pin2>
struct led
{
	typedef avrlib::pin<Port1, Pin1> pin1;
	typedef avrlib::pin<Port2, Pin2> pin2;

	led()
	{
		pin1::output(true);
		pin2::output(true);
	}

	~led()
	{
		pin1::output(false);
		pin2::output(false);
	}
	
	static void clear()
	{
		pin1::clear();
		pin2::clear();
	}

	static void green()
	{
		pin1::set();
		pin2::clear();
	}

	static void red()
	{
		pin1::clear();
		pin2::set();
	}
};

led<portc, 1, portc, 3> led0;
led<portc, 0, portc, 2> led1;
led<portc, 5, portc, 7> led2;
led<portc, 4, portc, 6> led3;


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

struct timer_t
{
	typedef uint32_t time_type;

	timer_t()
		: m_overflows(0), m_new_overflow(false)
	{
		TCCR0 = (1<<CS02)|(1<<CS01)|(1<<CS00);
		TIMSK |= (1<<TOIE0);
	}

	void process()
	{
		++m_overflows;
		m_new_overflow = true;
	}

	time_type operator()() const
	{
		uint8_t tcnt = TCNT0;
		for (;;)
		{
			uint16_t overflows = m_overflows;
			uint8_t tcnt2 = TCNT0;
			if (tcnt < tcnt2)
				return (time_type(overflows) << 8) + tcnt2;
			tcnt = tcnt2;
		}
	}

	bool new_overflow()
	{
		bool res = m_new_overflow;
		if (res)
			m_new_overflow = false;
		return res;
	}

	uint16_t volatile m_overflows;
	bool volatile m_new_overflow;
} timer;


struct repro_t
{
	void set()
	{
		OCR3A = 0x4000;
		TCCR3A = (1<<COM3A0);
		TCCR3B = (1<<WGM32)|(1<<CS30);
		DDRE |= (1<<3);
	}

	void clear()
	{
		DDRE &= ~(1<<3);
		TCCR3A = 0;
		TCCR3B = 0;
	}
};

template <typename Timer, typename Output>
class signaller_t
{
public:
	typedef typename Timer::time_type time_type;

	signaller_t(Timer const & timer, Output & out)
		: m_timer(timer), m_out(out), m_base(0), m_count(0), m_active(true)
	{
	}

	void signal(int count, time_type length = 4000, time_type space = 3000)
	{
		m_count = count;
		m_length = length;
		m_space = space;

		m_base = m_timer();

		m_active = true;
		m_out.set();
	}

	void process()
	{
		if (m_count == 0)
			return;

		time_type time = m_timer();
		if (!m_active && time - m_base > m_space)
		{
			m_out.set();
			m_active = true;
			m_base = time;
		}
		else if (m_active && time - m_base > m_length)
		{
			m_out.clear();
			m_active = false;
			m_base = time;
			--m_count;
		}
	}

private:
	Timer const & m_timer;
	Output & m_out;

	time_type m_base;
	int m_count;
	time_type m_length;
	time_type m_space;

	bool m_active;
};

async_usart<usart1, 128, 128, bootseq> rs232(38400);

repro_t repro;
signaller_t<timer_t, repro_t> signaller(timer, repro);

void process()
{
	signaller.process();
	rs232.process();
}

template <typename Timer>
void wait(Timer const & timer, typename Timer::time_type time)
{
	typename Timer::time_type base = timer();
	while (timer() - base < time)
	{
		process();
	}
}

ISR(TIMER0_OVF_vect)
{
	timer.process();
}

uint8_t from_hex_digit(uint8_t digit)
{
	if ('0' <= digit && digit <= '9')
		return digit - '0';
	return digit - 'a' + 10;
}

bool get_bt_addr(uint8_t addr[6])
{
	stopwatch<timer_t> st(timer);
	while (st() < 17000)
	{
		process();
		PORTC ^= (1<<5)|(1<<7);
	}

	send(rs232, "///");
	
	st.clear();
	while (st() < 17000)
	{
		process();
		PORTC ^= (1<<5)|(1<<7);
	}

	send(rs232, "AT*AILBA?\r");
	
	uint8_t buf[64];
	uint8_t buflen = 0;

	buflen = readline(rs232, buf, 32);
	buflen = readline(rs232, buf, 32);
	if (buflen != 21)
		return false;

	if (!bufcmp(buf, 7, "*AILBA:"))
		return false;

	for (uint8_t i = 0; i < 6; ++i)
	{
		addr[i] = from_hex_digit(buf[7 + i*2]) << 4;
		addr[i] |= from_hex_digit(buf[8 + i*2]);
	}

	send(rs232, "AT*ADDM\r");

	return true;
}

bool connect(uint8_t addr[6])
{
	stopwatch<timer_t> st(timer);
	while (st() < 17000)
	{
		process();
	}

	send(rs232, "///");
	
	st.clear();
	while (st() < 17000)
	{
		process();
	}

	send(rs232, "AT*ADNRP=1,0\r");
	send(rs232, "AT*ADDCP=0,0\r");
	send(rs232, "AT*ADWDRP=0,");
		
	for (int i = 0; i < 6; ++i)
	{
		char const digits[] = "0123456789abcdef";

		rs232.write(digits[addr[i] >> 4]);
		rs232.write(digits[addr[i] & 0x0f]);
	}

	send(rs232, ",2,0,\"\",0\r");
	send(rs232, "AT*ADDM\r");
	return true;
}

bool disconnect()
{
	stopwatch<timer_t> st(timer);
	while (st() < 17000)
	{
		process();
	}

	send(rs232, "///");
	
	st.clear();
	while (st() < 17000)
	{
		process();
	}

	send(rs232, "AT*ADNRP=0,0\r");
	send(rs232, "AT*ADDM\r");
	return true;
}

uint8_t get_target_no()
{
	uint8_t btn = get_buttons();
	return ((btn >> 4) & 1) | ((btn >> 5) & 2);
}

int main()
{
	sei();

	DDRB = 0;
	PORTB = 0xff;
	PORTF = 0;
	
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	int send_state = 0; // 0 -- silent, 1 -- text, 2 -- binary
	
	wait(timer, 1562);

	if (get_buttons() & (1<<0))
	{
		DDRC = (1<<5)|(1<<7);
		PORTC = (1<<5);

		wait(timer, 4000);
		signaller.signal(2);
		while (get_buttons() & (1<<0))
			process();

		wait(timer, 4000);
		signaller.signal(1);
		while ((get_buttons() & (1<<0)) == 0)
			process();

		wait(timer, 4000);
		signaller.signal(1);

		uint8_t addr[6];
		if (!get_bt_addr(addr))
		{
			PORTC = (1<<5);
		}
		else
		{
			PORTC = (1<<7);
			store_eeprom(1 + 6 * get_target_no(), addr, 6);
		}

		for (;;)
			process();
	}

	bool connected = false;

	timed_command_parser<timer_t> cmd_parser(timer, 2000);
	timeout<timer_t> led_timeout(timer, 15000);

	for (;;)
	{
		if (!connected && (get_buttons() & 4) != 0)
		{
			uint8_t addr[6];
			load_eeprom(1 + 6 * get_target_no(), addr, 6);
			connect(addr);
			connected = true;
		}

		if (connected && (get_buttons() & 4) == 0)
		{
			disconnect();
			connected = false;
		}

		if (!rs232.empty())
		{
			uint8_t ch = rs232.read();
			switch (cmd_parser.push_data(ch))
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
			case 'r':
				signaller.signal(3, 4000, 3000);
				break;
			case 'R':
				repro.clear();
				break;
			case 'p':
				{
					uint8_t addr[6];
					for (int j = 0; j < 4; ++j)
					{
						load_eeprom(1 + 6 * j, addr, 6);
						for (int i = 0; i < 6; ++i)
						{
							char const digits[] = "0123456789abcdef";

							rs232.write(digits[addr[i] >> 4]);
							rs232.write(digits[addr[i] & 0x0f]);
						}
						send(rs232, "\r\n");
					}
				}
				break;

			case 't':
				rs232.write('t');
				led0.red();
				break;

			case 'T':
				rs232.write('T');
				led0.green();
				break;

			case 8:
				if (cmd_parser.size() > 0)
				{
					if (cmd_parser[0] & (1<<0))
						led0.red();
					else
						led0.green();

					if (cmd_parser[0] & (1<<1))
						led1.red();
					else
						led1.green();

					if (cmd_parser[0] & (1<<2))
						led2.red();
					else
						led2.green();

					if (cmd_parser[0] & (1<<3))
						led3.red();
					else
						led3.green();
					led_timeout.clear();
				}
				break;

			case 255:
				break;

			default:
				send_state = 0;
			}
		}

		if (led_timeout)
		{
			led0.clear();
			led1.clear();
			led2.clear();
			led3.clear();
			led_timeout.clear();
		}

		if (timer.new_overflow())
		{
			if (connected || send_state == 2)
			{
				rs232.write(0x80);
				rs232.write(0x19);
				for (int i = 0; i < 4; ++i)
					send_bin(rs232, get_pot(i));
				send_bin(rs232, (uint8_t)(get_buttons() >> 4));
			}
			else if (send_state == 1)
			{
				for (int i = 0; i < 4; ++i)
					send_int(rs232, get_pot(i), 7);
				send_int(rs232, get_buttons(), 5);
				send(rs232, "\r\n");
			}
		}

		process();
	}
}
