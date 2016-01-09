// Hardware definitions for Jarek's transmitter

template <typename Port, int Pin>
struct led
	:led_base
{
	typedef avrlib::pin<Port, Pin> pin;

	led()
	{
		pin::output(true);
	}

	~led()
	{
		pin::output(false);
	}
	
	virtual void clear()
	{
		pin::clear();
	}

	virtual void green()
	{
		pin::set();
	}

	virtual void red()
	{
		pin::set();
	}
	
	virtual void toggle()
	{
		pin::toggle();
	}
};

led<porta, 3> led0;
led<porta, 2> led1;
led<porta, 1> led2;
led<porta, 3> led3;

led<porta, 3> led4;
led<porta, 2> led5;
led<porta, 1> led6;
led<porta, 3> led7;

pin    <porte, 2> sw0;
inv_pin<porte, 3> sw1;
inv_pin<porte, 4> sw2;
inv_pin<porte, 5> sw3;

inv_pin<porte, 7> sw4;
inv_pin<porte, 6> sw5;
pin    <portb, 2> sw6;
pin    <portb, 3> sw7;

static const uint8_t adc_channels = 5;

async_adc adcs[adc_channels] = {
	async_adc(1, true),
	async_adc(2, true),
	async_adc(0, true),
	async_adc(3, false),
	async_adc(6, false)
};

int16_t adc_offset[adc_channels] = { 17, -11, 14, -6, 0 };
int16_t adc_gain_pos[adc_channels] = { 96, 110, 83, 101, 1 };
int16_t adc_gain_neg[adc_channels] = { 107, 93, 122, 110, 1 };

uint8_t get_buttons()
{
	return make_byte(sw7.read(), sw0.read(), sw6.read(), sw1.read(), sw5.read(), sw2.read(), sw4.read(), sw3.read());
}

void hw_init()
{
	sw0.pullup();
	sw1.pullup();
	sw2.pullup();
	sw3.pullup();
	sw4.pullup();
	sw5.pullup();
	sw6.pullup();
	sw7.pullup();
}