// Hardware definitions for Yunibeer transmitter

template <typename Port1, int Pin1, typename Port2, int Pin2>
struct led
	:led_base
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
	
	virtual void clear()
	{
		pin1::clear();
		pin2::clear();
	}

	virtual void green()
	{
		pin1::set();
		pin2::clear();
	}

	virtual void red()
	{
		pin1::clear();
		pin2::set();
	}
	
	virtual void toggle()
	{
		pin1::toggle();
		pin2::toggle();
	}
};

led<porta, 7, portg, 0> led0;
led<porta, 6, portg, 1> led1;
led<porta, 1, porta, 3> led2;
led<porta, 0, porta, 2> led3;

led<portc, 5, portc, 7> led4;
led<portc, 4, portc, 6> led5;
led<portc, 1, portc, 3> led6;
led<portc, 0, portc, 2> led7;

pin<portb, 1> sw0;
pin<portb, 3> sw1;
pin<portb, 5> sw2;
pin<portb, 7> sw3;

pin<portb, 6> sw4;
pin<portb, 4> sw5;
pin<portb, 2> sw6;
pin<portb, 0> sw7;

static const uint8_t adc_channels = 5;

async_adc adcs[adc_channels] = {
	async_adc(4, true , false),
	async_adc(5, false, false),
	async_adc(6, false, false),
	async_adc(7, false, false),
	async_adc(0, false, true )
};

static const uint16_t low_battery_threshold = 39322;

void hw_init()
{
	DDRB = 0;
	PORTB = 0xff;
	PORTF = 0;
}