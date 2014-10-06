#include "avrlib/async_usart.hpp"
#include "avrlib/usart1.hpp"
#include "avrlib/bootseq.hpp"
#include "avrlib/format.hpp"
#include "avrlib/command_parser.hpp"
#include "avrlib/eeprom.hpp"
#include "avrlib/stopwatch.hpp"
#include "avrlib/make_byte.hpp"
#include "avrlib/adc.hpp"
#include "avrlib/math.hpp"

#include "avrlib/pin.hpp"
#include "avrlib/porta.hpp"
#include "avrlib/portb.hpp"
#include "avrlib/portc.hpp"
#include "avrlib/portg.hpp"

#include <string.h>
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
	
	static void toggle()
	{
		pin1::toggle();
		pin2::toggle();
	}
};

led<portc, 1, portc, 3> led6;//
led<portc, 0, portc, 2> led7;//
led<portc, 5, portc, 7> led4;//
led<portc, 4, portc, 6> led5;//

led<porta, 1, porta, 3> led2;//
led<porta, 0, porta, 2> led3;//
led<porta, 7, portg, 0> led0;//
led<porta, 6, portg, 1> led1;//

pin<portb, 1> sw0;
pin<portb, 3> sw1;
pin<portb, 5> sw2;
pin<portb, 7> sw3;

pin<portb, 6> sw4;
pin<portb, 4> sw5;
pin<portb, 2> sw6;
pin<portb, 0> sw7;

async_adc adcs[] = {
	async_adc(4, true),
	async_adc(5, false),
	async_adc(6, false),
	async_adc(7, false),
	async_adc(0, false),
};

uint8_t current_adc = 0;

int16_t get_pot(int index)
{
	return int16_t(adcs[index].value() - 0x8000);
}

uint8_t get_buttons()
{
	return PINB;
}

struct timer_t
{
	typedef uint32_t time_type;

	timer_t()
		: m_overflows(0)
	{
		TCCR0 = (1<<CS02)|(1<<CS01)|(1<<CS00);
		TIMSK |= (1<<TOIE0);
	}

	void process()
	{
		++m_overflows;
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
	
	inline time_type value() const { return this->operator()(); }

	uint16_t volatile m_overflows;
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

async_usart<usart1, 128, 128, bootseq> rs232(115200UL, false);

repro_t repro;
signaller_t<timer_t, repro_t> signaller(timer, repro);

void process()
{
	signaller.process();
	rs232.process_rx();
	rs232.process_tx();
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

template <typename Stream>
void send_lego_header(Stream & s, char const * title, const uint16_t& value_length)
{
	uint8_t title_len = strlen(title) + 1; // +1 -> terminating null character
	uint16_t msg_len = 4 + 1 + title_len + 2 + value_length;
	// 16b packet length
	send_bin(s, msg_len);
	// const header
	s.write(0x01);
	s.write(0x00);
	s.write(0x81);
	s.write(0x9e);
	// title length
	s.write(title_len);
	// title
	send(s, title);
	s.write(0);
	// value size
	send_bin(s, value_length);
}

template <typename Stream>
void send_lego(Stream & s, char const * title, const bool& value)
{
	send_lego_header(s, title, 1);
	send_bin(s, uint8_t(value));
}

template <typename Stream>
void send_lego(Stream & s, char const * title, const float& value)
{
	send_lego_header(s, title, 4);
	send_bin(s, value);
}

template <typename Stream>
void send_lego(Stream & s, char const * title, char const * value)
{
	uint16_t value_length = strlen(value) + 1;
	send_lego_header(s, title, value_length);
	send(s, value);
	s.write(0);
}

void sw_test()
{
	send(rs232, "sw0-7:\n");
	send(rs232, sw0.read() ? "1" : "0");
	send(rs232, sw1.read() ? "1" : "0");
	send(rs232, sw2.read() ? "1" : "0");
	send(rs232, sw3.read() ? "1" : "0");
	send(rs232, sw4.read() ? "1" : "0");
	send(rs232, sw5.read() ? "1" : "0");
	send(rs232, sw6.read() ? "1" : "0");
	send(rs232, sw7.read() ? "1" : "0");
	send(rs232, "\n");
	send(rs232, "get_buttons:\n");
	send_bin_text(rs232, get_buttons(), 8);
	send(rs232, "\n");
	send(rs232, "get_target_no:\n");
	send_int(rs232, get_target_no());
	send(rs232, "\n");
	rs232.flush();
}

void led_test()
{
	uint32_t wait_time = 16384>>1;
	
	led0.green();
	wait(timer, wait_time);
	led0.red();
	wait(timer, wait_time);
	led0.clear();
	
	led1.green();
	wait(timer, wait_time);
	led1.red();
	wait(timer, wait_time);
	led1.clear();
	
	led2.green();
	wait(timer, wait_time);
	led2.red();
	wait(timer, wait_time);
	led2.clear();
	
	led3.green();
	wait(timer, wait_time);
	led3.red();
	wait(timer, wait_time);
	led3.clear();
	
	led4.green();
	wait(timer, wait_time);
	led4.red();
	wait(timer, wait_time);
	led4.clear();
	
	led5.green();
	wait(timer, wait_time);
	led5.red();
	wait(timer, wait_time);
	led5.clear();
	
	led6.green();
	wait(timer, wait_time);
	led6.red();
	wait(timer, wait_time);
	led6.clear();
	
	led7.green();
	wait(timer, wait_time);
	led7.red();
	wait(timer, wait_time);
	led7.clear();
}

int main()
{
	sei();

	DDRB = 0;
	PORTB = 0xff;
	PORTF = 0;
	
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	
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
	
	int send_state = get_target_no() + 1; // 0 -- silent, 1 -- text, 2 -- binary, 3 -- PIC interface, 4 -- LEGO protocol
	uint32_t data_send_timeout_time = 256; // 16.384ms
	
	switch(send_state)
	{
	case 1:
		led0.green();
		break;
	case 2:
		led1.green();
		break;
	case 3:
		led2.green();
		break;
	case 4:
		led3.green();
		data_send_timeout_time = 256*3; // 16.384ms * 3
		break;
	}

	bool connected = false;
	bool force_send = false;

	timed_command_parser<timer_t> cmd_parser(timer, 2000);
	timeout<timer_t> led_timeout(timer, 7000);
	led_timeout.cancel();

	timeout<timer_t> low_battery_timeout(timer, 300000);
	timeout<timer_t> data_send_timeout(timer, data_send_timeout_time);

	signaller.signal(1, 1500);
	
	wait(timer, 8192, process);
	led0.clear();
	led1.clear();
	led2.clear();
	led3.clear();

	int32_t cnt = 0;

	for (;;)
	{
		if (!connected && (get_buttons() & 4) != 0)
		{
			uint8_t addr[6];
			load_eeprom(1 + 6 * get_target_no(), addr, 6);
			connect(addr);
			connected = true;
			cnt = 0;
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
			case 'n':
				rs232.write('\n');
				break;
			case '1':
				send(rs232, "text\n");
				send_state = 1;
				force_send = true;
				break;
			case '2':
				send(rs232, "bin\n");
				send_state = 2;
				PORTC ^= (1<<5)|(1<<7);
				force_send = true;
				break;
			case '3':
				send(rs232, "PIC\n");
				send_state = 3;
				force_send = true;
				break;
			case '4':
				send(rs232, "LEGO\n");
				send_state = 4;
				force_send = true;
				break;
			case '?':
				force_send = false;
				send(rs232, "'1' -- text, '2' -- binary, 3 -- PIC interface, 4 -- LEGO protocol\r\n");
				format(rs232, "selected: % \n") % send_state;
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
			case 'P':
			{
				uint8_t mac[6] = {0};
				send(rs232, "insert address index (0 - 3): ");
				rs232.flush();
				uint8_t addr = rs232.read()-'0';
				if(addr > 3)
				{
					send(rs232, "invalid position\n");
					rs232.flush();
					break;
				}
				format(rs232, "% \ninsert address: ") % addr;
				rs232.flush();
				for(uint8_t i = 0; i != 12; ++i)
				{
					uint8_t digit = rs232.read();
					if(digit >= '0' && digit <= '9')
						mac[i>>1] |= digit - '0';
					else if(digit >= 'a' && digit <= 'f')
						mac[i>>1] |= digit - 'a' + 10;
					else if(digit >= 'A' && digit <= 'F')
						mac[i>>1] |= digit - 'A' + 10;
					else
					{
						send(rs232, " invalid character\n");
						rs232.flush();
						addr = 255;
						break;
					}
					if((i & 1) == 0)
						mac[i>>1] <<= 4;
					rs232.write(digit);
					rs232.flush();
				}
				if(addr != 255)
					store_eeprom(1 + 6 * addr, mac, 6);
				send(rs232, "\ndone\n\n");
				rs232.flush();
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

			case 'b':
				send_int(rs232, adcs[4].value());
				send(rs232, "\r\n");
				break;
				
			case 's':
				sw_test();
				break;
			
			case 'l':
				led_test();
				break;

			case 8:
				if (cmd_parser.size() > 0)
				{
					if (cmd_parser[0] & (1<<0))
						led4.red();
					else
						led4.green();

					if (cmd_parser[0] & (1<<1))
						led5.red();
					else
						led5.green();

					if (cmd_parser[0] & (1<<2))
						led6.red();
					else
						led6.green();

					if (cmd_parser[0] & (1<<3))
						led7.red();
					else
						led7.green();
					led_timeout.restart();
				}
				break;

			case 255:
				break;

			default:
				force_send = false;
			}
		}

		if (led_timeout)
		{
			led4.clear();
			led5.clear();
			led6.clear();
			led7.clear();

			if (connected)
				signaller.signal(5);

			led_timeout.cancel();
		}

		if (data_send_timeout)
		{
			data_send_timeout.restart();

			if(connected || force_send)
			{
				switch(send_state)
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
					send_bin(rs232, make_byte(sw0.value(), sw1.value(), sw2.value(), sw3.value(), sw4.value(), sw5.value(), 0, sw7.value()));
					break;
				case 3:
					rs232.write(0xFF);
					for (int i = 0; i < 4; ++i)
						send_bin(rs232, avrlib::clamp(uint8_t(128+(get_pot(i)>>8)), 0, 254));
					break;
				case 4:
					send_lego(rs232, "a0", float(-get_pot(0)/32767.0));
					send_lego(rs232, "a1", float( get_pot(1)/32767.0));
					send_lego(rs232, "a2", float( get_pot(2)/32767.0));
					send_lego(rs232, "a3", float( get_pot(3)/32767.0));
					send_lego(rs232, "b0", sw0.read());
					send_lego(rs232, "b1", sw1.read());
					send_lego(rs232, "b2", sw2.read());
					send_lego(rs232, "b3", sw3.read());
					send_lego(rs232, "cnt", float(cnt));
					++cnt;
					break;
				}
			}
		}

		if (adcs[current_adc].process())
		{
			if (++current_adc == 5)
				current_adc = 0;

			adcs[current_adc].start();
		}

		if (adcs[4].value() < 39322 && low_battery_timeout)
		{
			signaller.signal(3, 1500, 1000);
			low_battery_timeout.restart();
		}

		process();
	}
}
